// doc: https://www.openfirmware.info/data/docs/bus.pci.pdf

use alloc::string::String;
use alloc::vec::Vec;
use core::iter::Iterator;
use core::mem::size_of;
use core::{str, u32, u64, u8};

use arm_gic::gicv3::{IntId, Trigger};
use bit_field::BitField;
use hermit_dtb::Dtb;
use pci_types::{
	Bar, ConfigRegionAccess, EndpointHeader, HeaderType, InterruptLine, InterruptPin, PciAddress,
	PciHeader, MAX_BARS,
};

use crate::arch::aarch64::kernel::interrupts::GIC;
use crate::arch::aarch64::mm::paging::{self, BasePageSize, PageSize, PageTableEntryFlags};
use crate::arch::aarch64::mm::{virtualmem, PhysAddr, VirtAddr};
use crate::drivers::pci::{PciCommand, PciDevice, PCI_DEVICES};
use crate::kernel::boot_info;

const PCI_MAX_DEVICE_NUMBER: u8 = 32;
const PCI_MAX_FUNCTION_NUMBER: u8 = 8;

const DTB_CELL_SIZE: usize = size_of::<u32>();

pub static mut PCI_ROOT_ADDR: VirtAddr = VirtAddr::zero();

#[derive(Debug, Copy, Clone)]
pub struct PciConfigRegion(VirtAddr);

pub struct PciConfigRegionIter {
	/*
	cfg_offset(bus, device, function, register) =
				   bus << 20 | device << 15 | function << 12 | register
	 */
	bus_end: u8,
	end_of_iteration: bool,

	//next to be probed
	bus: u8, //8bits
	dev: u8, //5bits
	         // register: 8bits
}

impl PciConfigRegionIter {
	pub fn new(bus_end: u8) -> Self {
		Self {
			bus_end,
			end_of_iteration: false,
			bus: 0,
			dev: 0,
		}
	}
}

impl Iterator for PciConfigRegionIter {
	type Item = (PciAddress, u8, u8, u8); //bus, dev, function

	fn next(&mut self) -> Option<Self::Item> {
		if self.end_of_iteration {
			return None;
		}

		//currently only single-function device is supported
		let probe_addr = PciAddress::new(0, self.bus, self.dev, 0);
		let bus_copy = self.bus;
		let dev_copy = self.dev;

		if self.bus == self.bus_end && self.dev == 0b11111 {
			self.end_of_iteration = true;
		} else if self.dev < 0b11111 {
			self.dev += 1;
		} else {
			self.bus += 1;
			self.dev = 0;
		}

		return Some((probe_addr, bus_copy, dev_copy, 0));
	}
}

impl PciConfigRegion {
	pub const fn new(addr: VirtAddr) -> Self {
		// in qemu, the pci dtb compatible property is 'pci-host-ecam-generic'
		// which requires the configuration space base address to be aligned to 28bits
		// check https://www.kernel.org/doc/Documentation/devicetree/bindings/pci/host-generic-pci.txt
		assert!(addr.as_u64() & 0xFFFFFFF == 0, "Unaligned PCI Config Space");
		Self(addr)
	}

	#[inline]
	fn addr_from_offset(&self, pci_addr: PciAddress, offset: u16) -> usize {
		assert!(offset & 0xF000 == 0, "Invalid offset");
		(u64::from(pci_addr.bus()) << 20
			| u64::from(pci_addr.device()) << 15
			| u64::from(pci_addr.function()) << 12
			| (u64::from(offset) & 0xFFF)
			| self.0.as_u64()) as usize // this is the base address
	}
}

impl ConfigRegionAccess for PciConfigRegion {
	#[inline]
	fn function_exists(&self, _address: PciAddress) -> bool {
		// we trust the device tree
		true
	}

	#[inline]
	unsafe fn read(&self, pci_addr: PciAddress, offset: u16) -> u32 {
		let ptr = core::ptr::from_exposed_addr(self.addr_from_offset(pci_addr, offset));
		unsafe { crate::drivers::pci::from_pci_endian(core::ptr::read_volatile(ptr)) }
	}

	#[inline]
	unsafe fn write(&self, pci_addr: PciAddress, offset: u16, value: u32) {
		let ptr = core::ptr::from_exposed_addr_mut(self.addr_from_offset(pci_addr, offset));
		unsafe {
			core::ptr::write_volatile(ptr, value.to_le());
		}
	}
}

/// Try to find regions for the device registers
#[allow(unused_assignments, dead_code)]
fn detect_pci_regions(dtb: &Dtb<'_>, parts: &[&str]) -> (u64, u64, u64) {
	let mut io_start: u64 = 0;
	let mut mem32_start: u64 = 0;
	let mut mem64_start: u64 = 0;

	let mut residual_slice = dtb.get_property(parts.first().unwrap(), "ranges").unwrap();
	let mut value_slice;
	while !residual_slice.is_empty() {
		(value_slice, residual_slice) = residual_slice.split_at(core::mem::size_of::<u32>());
		let high = u32::from_be_bytes(value_slice.try_into().unwrap());
		(value_slice, residual_slice) = residual_slice.split_at(core::mem::size_of::<u32>());
		let _mid = u32::from_be_bytes(value_slice.try_into().unwrap());
		(value_slice, residual_slice) = residual_slice.split_at(core::mem::size_of::<u32>());
		let _low = u32::from_be_bytes(value_slice.try_into().unwrap());

		match high.get_bits(24..=25) {
			0b00 => debug!("Configuration space"),
			0b01 => {
				debug!("IO space");
				if io_start != 0 {
					warn!("Found already IO space");
				}

				(value_slice, residual_slice) =
					residual_slice.split_at(core::mem::size_of::<u64>());
				io_start = u64::from_be_bytes(value_slice.try_into().unwrap());
			}
			0b10 => {
				let prefetchable = high.get_bit(30);
				debug!("32 bit memory space: prefetchable {}", prefetchable);
				if mem32_start != 0 {
					warn!("Found already 32 bit memory space");
				}

				(value_slice, residual_slice) =
					residual_slice.split_at(core::mem::size_of::<u64>());
				mem32_start = u64::from_be_bytes(value_slice.try_into().unwrap());
			}
			0b11 => {
				let prefetchable = high.get_bit(30);
				debug!("64 bit memory space: prefetchable {}", prefetchable);
				if mem64_start != 0 {
					warn!("Found already 64 bit memory space");
				}

				(value_slice, residual_slice) =
					residual_slice.split_at(core::mem::size_of::<u64>());
				mem64_start = u64::from_be_bytes(value_slice.try_into().unwrap());
			}
			_ => panic!("Unknown space code"),
		}

		// currently, the size is ignores
		(value_slice, residual_slice) = residual_slice.split_at(core::mem::size_of::<u64>());
		//let size = u64::from_be_bytes(value_slice.try_into().unwrap());
	}

	(io_start, mem32_start, mem64_start)
}

#[allow(unused_assignments)]
fn detect_pci_regions2(dtb: &Dtb<'_>, parts: &[&str]) -> (u64, u64, u64) {
	let size = dtb.get_property("/", "#address-cells").unwrap();
	let parent_address_cells = u32::from_be_bytes(size.try_into().unwrap()) as usize;
	let size = dtb
		.get_property(parts.first().unwrap(), "#address-cells")
		.unwrap();
	let child_address_cells = u32::from_be_bytes(size.try_into().unwrap()) as usize;
	let size = dtb
		.get_property(parts.first().unwrap(), "#size-cells")
		.unwrap();
	let child_size_cells = u32::from_be_bytes(size.try_into().unwrap()) as usize;
	info!("parent address cells: {:#04x}", parent_address_cells);
	info!("child address cells: {:#04x}", child_address_cells);
	info!("child size cells: {:#04x}", child_size_cells);

	let chunk_size: usize =
		(child_address_cells + parent_address_cells + child_size_cells) * size_of::<u32>();

	let pcie_ranges = dtb.get_property(parts.first().unwrap(), "ranges").unwrap();
	assert!(pcie_ranges.len() % chunk_size == 0);

	let mut io_start: u64 = 0;
	let mut mem32_start: u64 = 0;
	let mut mem64_start: u64 = 0;

	let pw = (2 * parent_address_cells * size_of::<u32>()) as usize;

	for chunk in pcie_ranges.chunks(chunk_size) {
		let (child_address, rest) = chunk.split_at(child_address_cells * size_of::<u32>());
		let (parent_address, region_size) = rest.split_at(parent_address_cells * size_of::<u32>());
		let region_size = u64::from_be_bytes(region_size.try_into().unwrap());

		let (bitfield, child_address) = child_address.split_at(size_of::<u32>());
		info!(
			"PCI address: 0x{:x}",
			u64::from_be_bytes(child_address.try_into().unwrap())
		);
		assert!((bitfield[0] & 0b01000000) == 0);
		info!(
			"The region is non-prefetchable with size 0x{:x}",
			region_size
		);

		match bitfield[0] & 0b11 {
			0b00 => debug!("configuration space"),
			0b01 => {
				if io_start != 0 {
					panic!("io_start has been set");
				}
				io_start = u64::from_be_bytes(parent_address.try_into().unwrap());
				info!("I/O space: 0x{:0pw$x}", io_start);
			}
			0b10 => {
				if mem32_start != 0 {
					panic!("mem32_start has been set");
				}
				mem32_start = u64::from_be_bytes(parent_address.try_into().unwrap());
				info!("32bit memory space: 0x{:0pw$x}", mem32_start);
			}
			0b11 => {
				if mem64_start != 0 {
					panic!("mem64_start has been set");
				}
				mem64_start = u64::from_be_bytes(parent_address.try_into().unwrap());
				info!("64bit memory space: 0x{:0pw$x}", mem64_start);
			}
			_ => panic!("unknown space code"),
		}

		/*
		info!(
			"child_address:0x{:0pw$x},parent_address:0x{:0pw$x}, size:{}",
			u64::from_be_bytes(child_address.try_into().unwrap()),
			u64::from_be_bytes(parent_address.try_into().unwrap()),
			u64::from_be_bytes(region_size.try_into().unwrap()),
		);
		*/
	}
	(io_start, mem32_start, mem64_start)
}
#[allow(unused_assignments, dead_code)]
fn detect_interrupt(
	bus: u32,
	dev: u32,
	dtb: &Dtb<'_>,
	parts: &[&str],
) -> Option<(InterruptPin, InterruptLine)> {
	let addr = bus << 16 | dev << 11;
	if addr == 0 {
		// assume PCI bridge => no configuration required
		return None;
	}

	let mut pin: u8 = 0;

	//let slice = dtb.get_property("/", "interrupt-parent").unwrap();
	//let interrupt_parent = u32::from_be_bytes(slice.try_into().unwrap());

	let slice = dtb.get_property("/", "#address-cells").unwrap();
	let address_cells = u32::from_be_bytes(slice.try_into().unwrap());

	//let slice = dtb.get_property("/intc", "#interrupt-cells").unwrap();
	//let interrupt_cells = u32::from_be_bytes(slice.try_into().unwrap());

	let mut residual_slice = dtb
		.get_property(parts.first().unwrap(), "interrupt-map")
		.unwrap();
	let mut value_slice;
	while !residual_slice.is_empty() {
		(value_slice, residual_slice) = residual_slice.split_at(core::mem::size_of::<u32>());
		let high = u32::from_be_bytes(value_slice.try_into().unwrap());
		(value_slice, residual_slice) = residual_slice.split_at(core::mem::size_of::<u32>());
		let _mid = u32::from_be_bytes(value_slice.try_into().unwrap());
		(value_slice, residual_slice) = residual_slice.split_at(core::mem::size_of::<u32>());
		let _low = u32::from_be_bytes(value_slice.try_into().unwrap());

		(value_slice, residual_slice) = residual_slice.split_at(core::mem::size_of::<u32>());
		//let child_specifier = u32::from_be_bytes(value_slice.try_into().unwrap());

		(value_slice, residual_slice) = residual_slice.split_at(core::mem::size_of::<u32>());
		//let parent = u32::from_be_bytes(value_slice.try_into().unwrap());

		for _i in 0..address_cells {
			(value_slice, residual_slice) = residual_slice.split_at(core::mem::size_of::<u32>());
			//let parent_address = u32::from_be_bytes(value_slice.try_into().unwrap());
		}

		// The 1st cell is the interrupt type; 0 for SPI interrupts, 1 for PPI
		// interrupts.
		(value_slice, residual_slice) = residual_slice.split_at(core::mem::size_of::<u32>());
		let irq_type = u32::from_be_bytes(value_slice.try_into().unwrap());

		// The 2nd cell contains the interrupt number for the interrupt type.
		// SPI interrupts are in the range [0-987].  PPI interrupts are in the
		// range [0-15].
		(value_slice, residual_slice) = residual_slice.split_at(core::mem::size_of::<u32>());
		let irq_number = u32::from_be_bytes(value_slice.try_into().unwrap());

		// The 3rd cell is the flags, encoded as follows:
		// bits[3:0] trigger type and level flags.
		//		1 = low-to-high edge triggered
		// 		2 = high-to-low edge triggered (invalid for SPIs)
		//		4 = active high level-sensitive
		//		8 = active low level-sensitive (invalid for SPIs).
		// bits[15:8] PPI interrupt cpu mask.  Each bit corresponds to each of
		// the 8 possible cpus attached to the GIC.  A bit set to '1' indicated
		// the interrupt is wired to that CPU.  Only valid for PPI interrupts.
		// Also note that the configurability of PPI interrupts is IMPLEMENTATION
		// DEFINED and as such not guaranteed to be present (most SoC available
		// in 2014 seem to ignore the setting of this flag and use the hardware
		// default value).
		(value_slice, residual_slice) = residual_slice.split_at(core::mem::size_of::<u32>());
		let irq_flags = u32::from_be_bytes(value_slice.try_into().unwrap());

		trace!(
			"Interrupt type {:#x}, number {:#x} flags {:#x}",
			irq_type,
			irq_number,
			irq_flags
		);

		if high.get_bits(0..24) == addr {
			pin += 1;
			if irq_type == 0 {
				// enable interrupt
				let irq_id = IntId::spi(irq_number);
				let gic = unsafe { GIC.get_mut().unwrap() };
				gic.set_interrupt_priority(irq_id, 0x10);
				if irq_flags == 4 {
					gic.set_trigger(irq_id, Trigger::Level);
				} else if irq_flags == 2 {
					gic.set_trigger(irq_id, Trigger::Edge);
				} else {
					panic!("Invalid interrupt level!");
				}
				gic.enable_interrupt(irq_id, true);

				return Some((pin, irq_number.try_into().unwrap()));
			}
		}
	}

	None
}

#[derive(Copy, Clone, Debug)]
struct PciInterruptData {
	dev_num: u8,
	dev_id: u16,
	dev_pin: u8,
	irq_type: u8, // SPI:0, PPI: 1
	irq_num: u32,
	trigger: Trigger,
}
struct InterruptMapChunks<'a> {
	map: &'a [u8],
}

impl<'a> InterruptMapChunks<'a> {
	fn query_device(&self, dev_num: u8, dev_id: u16) -> Option<PciInterruptData> {
		for rest in self.map.chunks(10 * DTB_CELL_SIZE) {
			let (pci_address, rest) = rest.split_at(3 * DTB_CELL_SIZE);
			let (pin, rest) = rest.split_at(1 * DTB_CELL_SIZE);
			let (_, rest) = rest.split_at(3 * DTB_CELL_SIZE);
			let (interrupt_controller_data, rest) = rest.split_at(3 * DTB_CELL_SIZE);
			assert!(rest.is_empty());
			assert!(interrupt_controller_data.len() == 3 * DTB_CELL_SIZE);

			let (bitfield, _) = pci_address.split_at(DTB_CELL_SIZE);
			let bitfield = u32::from_be_bytes(bitfield.try_into().unwrap());
			if dev_num != ((bitfield & 0x1800) >> 11).try_into().unwrap() {
				continue;
			} else {
				let dev_pin = match u32::from_be_bytes(pin.try_into().unwrap()) & 0x7 {
					0b01 => 1,
					0b10 => 2,
					0b11 => 3,
					0b100 => 4,
					_ => return None,
				};

				let (c1, interrupt_controller_data) =
					interrupt_controller_data.split_at(DTB_CELL_SIZE);
				let (c2, interrupt_controller_data) =
					interrupt_controller_data.split_at(DTB_CELL_SIZE);
				let (c3, interrupt_controller_data) =
					interrupt_controller_data.split_at(DTB_CELL_SIZE);
				let irq_type = u32::from_be_bytes(c1.try_into().unwrap());
				if irq_type > 1 {
					return None;
				}
				let irq_type = irq_type.try_into().unwrap();
				let irq_num = u32::from_be_bytes(c2.try_into().unwrap());

				let trigger = match u32::from_be_bytes(c3.try_into().unwrap()) {
					1u32 => Trigger::Edge,
					4u32 => Trigger::Level,
					_ => return None,
				};
				return Some(PciInterruptData {
					dev_num,
					dev_id,
					dev_pin,
					irq_type,
					irq_num,
					trigger,
				});
			}
		}
		None
	}
}

// a lot of assumptions are hardcoded...
// do a safety check before applying them
fn safety_check_interrupt_map<'a>(dtb: &Dtb<'a>, pcie: &str) -> InterruptMapChunks<'a> {
	//see https://michael2012z.medium.com/understanding-pci-node-in-fdt-769a894a13cc
	//for interrupt_controller_data, see https://www.kernel.org/doc/Documentation/devicetree/bindings/interrupt-controller/arm%2Cgic-v3.txt
	//detect interrupt controller
	let phandle = dtb.get_property("/", "interrupt-parent").unwrap();
	let phandle = u32::from_be_bytes(phandle.try_into().unwrap());
	info!("phandle: 0x{:x}", phandle);

	let mut intc = String::new();

	for node in dtb.enum_subnodes("/") {
		let parts: Vec<_> = node.split('@').collect();

		if let Some(p) = dtb.get_property(parts.first().unwrap(), "phandle") {
			if phandle == u32::from_be_bytes(p.try_into().unwrap()) {
				intc.push_str(parts.first().unwrap());
				break;
			}
		}
	}

	let intc = intc.as_str();

	let parent_unit_address_len = u32::from_be_bytes(
		dtb.get_property(intc, "#address-cells")
			.unwrap()
			.try_into()
			.unwrap(),
	) as usize; // 0
	assert!(parent_unit_address_len == 2);
	let parent_specifier_len = u32::from_be_bytes(
		dtb.get_property(intc, "#interrupt-cells")
			.unwrap()
			.try_into()
			.unwrap(),
	) as usize; //interrupt controller data
	assert!(parent_specifier_len == 3);
	let child_unit_address_len = u32::from_be_bytes(
		dtb.get_property(pcie, "#address-cells")
			.unwrap()
			.try_into()
			.unwrap(),
	) as usize; //PCI address
	assert!(child_unit_address_len == 3);
	let child_specifier_len = u32::from_be_bytes(
		dtb.get_property(pcie, "#interrupt-cells")
			.unwrap()
			.try_into()
			.unwrap(),
	) as usize; //PCI device pin
	assert!(child_specifier_len == 1);

	info!(
		"child_unit: {}, child_specifier:{}\nparent_unit: {}, parent_specifier: {}",
		child_unit_address_len, child_specifier_len, parent_unit_address_len, parent_specifier_len
	);

	let map_entry_len = DTB_CELL_SIZE
		* (child_unit_address_len
			+ child_specifier_len
			+ 1usize + parent_unit_address_len
			+ parent_specifier_len);

	let map_mask = dtb.get_property(pcie, "interrupt-map-mask").unwrap();
	assert!(map_mask.len() == DTB_CELL_SIZE * (child_unit_address_len + child_specifier_len));

	let (high, rest) = map_mask.split_at(DTB_CELL_SIZE);
	let (mid, rest) = rest.split_at(DTB_CELL_SIZE);
	let (low, rest) = rest.split_at(DTB_CELL_SIZE);
	let (end, rest) = rest.split_at(DTB_CELL_SIZE);
	assert!(rest.is_empty());
	let (high_mask, mid_mask, low_mask, pin_mask) = (
		u32::from_be_bytes(high.try_into().unwrap()),
		u32::from_be_bytes(mid.try_into().unwrap()),
		u32::from_be_bytes(low.try_into().unwrap()),
		u32::from_be_bytes(end.try_into().unwrap()),
	);
	assert!(high_mask == 0x1800);
	assert!(mid_mask == 0);
	assert!(low_mask == 0);
	assert!(pin_mask == 0x7);

	//construct mapping table
	if let Some(map) = dtb.get_property(pcie, "interrupt-map") {
		if map.len() % map_entry_len != 0 {
			panic!("wrong interrupt map len: {}", map.len());
		}
		for rest in map.chunks(map_entry_len) {
			let (child_unit_address, rest) = rest.split_at(child_unit_address_len * DTB_CELL_SIZE);
			let (child_interrupt_specifier, rest) =
				rest.split_at(child_specifier_len * DTB_CELL_SIZE);
			let (ph, rest) = rest.split_at(DTB_CELL_SIZE);
			if u32::from_be_bytes(ph.try_into().unwrap()) != phandle {
				panic!("wrong phandle: {:?}", ph)
			}
			let (parent_unit_address, rest) =
				rest.split_at(parent_unit_address_len * DTB_CELL_SIZE);
			let (parent_interrupt_specifier, rest) =
				rest.split_at(parent_specifier_len * DTB_CELL_SIZE);
			assert!(rest.is_empty());

			let parent_unit = u64::from_be_bytes(parent_unit_address.try_into().unwrap());
			assert!(parent_unit == 0);
		}
		return InterruptMapChunks { map };
	} else {
		panic!("pcie does not contain an interrupt-map");
	};
}
pub fn init() {
	let dtb = unsafe {
		Dtb::from_raw(core::ptr::from_exposed_addr(
			boot_info().hardware_info.device_tree.unwrap().get() as usize,
		))
		.expect(".dtb file has invalid header")
	};

	for node in dtb.enum_subnodes("/") {
		let parts: Vec<_> = node.split('@').collect();

		if let Some(compatible) = dtb.get_property(parts.first().unwrap(), "compatible") {
			if str::from_utf8(compatible)
				.unwrap()
				.contains("pci-host-ecam-generic")
			{
				let reg = dtb.get_property(parts.first().unwrap(), "reg").unwrap();
				let (slice, residual_slice) = reg.split_at(core::mem::size_of::<u64>());
				let addr = PhysAddr(u64::from_be_bytes(slice.try_into().unwrap()));
				let (slice, _residual_slice) = residual_slice.split_at(core::mem::size_of::<u64>());
				let size = u64::from_be_bytes(slice.try_into().unwrap());

				let bus_range = dtb
					.get_property(parts.first().unwrap(), "bus-range")
					.unwrap();
				let (bus_start, bus_end) = bus_range.split_at(bus_range.len() / 2);
				let bus_end = u32::from_be_bytes(bus_end.try_into().unwrap());

				info!("PCI configuration space physical address: 0x{:p}", addr);

				let pci_address =
					virtualmem::allocate_aligned(size.try_into().unwrap(), 0x10000000).unwrap();
				info!("Mapping PCI Enhanced Configuration Space interface to virtual address 0x{:p} (size {:#X})", pci_address, size);

				let mut flags = PageTableEntryFlags::empty();
				flags.device().writable().execute_disable();
				paging::map::<BasePageSize>(
					pci_address,
					addr,
					(size / BasePageSize::SIZE).try_into().unwrap(),
					flags,
				);

				unsafe {
					PCI_ROOT_ADDR = pci_address;
				}

				let (mut io_start, mem32_start, mut mem64_start) =
					detect_pci_regions2(&dtb, &parts);

				assert!(io_start > 0);
				assert!(mem32_start > 0);
				assert!(mem64_start > 0);

				let map_chunks = safety_check_interrupt_map(&dtb, parts.first().unwrap());

				let pci_config = PciConfigRegion::new(pci_address);

				let mut mem32_allocated: u32 = mem32_start.try_into().unwrap();
				let align_up =
					|value: u32, alignment: u32| -> u32 { ((value - 1) | (alignment - 1)) + 1 };

				//scan PCIE
				let pci_iter = PciConfigRegionIter::new(bus_end.try_into().unwrap());
				for (pa, _bus_num, _dev_num, _function_num) in pci_iter {
					let pd = PciHeader::new(pa);
					let (vendor_id, device_id) = pd.id(&pci_config);
					let (header, has_multiple_functions) = (
						pd.header_type(&pci_config),
						pd.has_multiple_functions(&pci_config),
					);
					if device_id == 0xFFFF {
						continue;
					}

					if has_multiple_functions {
						panic!(
							"multiple function pcie device is not supported, device_id: 0x{:x}",
							device_id
						);
					}
					if header != HeaderType::Endpoint {
						panic!("non endpoint header type is not supported");
					}

					let pd_type0 = EndpointHeader::from_header(pd, &pci_config).unwrap();
					let (pin, line) = pd_type0.interrupt(&pci_config);
					info!("bus number: {}, device number: {},device id: 0x{:x}, vendor id: 0x{:x}, header: {:?}, has_multiple_functions: {}, pin: {}, line: {}",pa.bus(),pa.device(), device_id, vendor_id, header, has_multiple_functions, pin, line);

					let dev = PciDevice::new(pa, pci_config);
					// Initializes BARs
					let mut cmd = PciCommand::default();
					let mut bar_index = 0usize;
					while bar_index < MAX_BARS {
						if let Some(bar) = dev.get_bar(bar_index.try_into().unwrap()) {
							match bar {
								Bar::Io { .. } => {
									info!("io bar:{}", bar_index);
									dev.set_bar(
										bar_index.try_into().unwrap(),
										Bar::Io {
											port: io_start.try_into().unwrap(),
										},
									);
									io_start += 0x20;
									cmd |=
										PciCommand::PCI_COMMAND_IO | PciCommand::PCI_COMMAND_MASTER;
									bar_index += 1;
								}
								Bar::Memory32 {
									address,
									size,
									prefetchable,
								} => {
									info!(
										"mem32 size: 0x{:x}, bar: {}, dev: 0x{:x}",
										size, bar_index, device_id
									);
									mem32_allocated = align_up(mem32_allocated, size);
									dev.set_bar(
										bar_index.try_into().unwrap(),
										Bar::Memory32 {
											address: mem32_allocated,
											size,
											prefetchable,
										},
									);
									mem32_allocated += size;
									cmd |= PciCommand::PCI_COMMAND_MEMORY
										| PciCommand::PCI_COMMAND_MASTER;
									bar_index += 1;
								}
								Bar::Memory64 {
									address: _,
									size,
									prefetchable,
								} => {
									info!(
										"mem64 size: 0x{:x}, bar: {}, dev: 0x{:x}",
										size, bar_index, device_id
									);
									dev.set_bar(
										bar_index.try_into().unwrap(),
										Bar::Memory64 {
											address: mem64_start,
											size,
											prefetchable,
										},
									);
									mem64_start += size;
									cmd |= PciCommand::PCI_COMMAND_MEMORY
										| PciCommand::PCI_COMMAND_MASTER;
									bar_index += 2;
								}
								_ => {}
							}
						} else {
							bar_index += 1;
						}
					}
					dev.set_command(cmd);

					let (bus, device_number) = (pa.bus(), pa.device());

					if let Some(irq_data) = map_chunks.query_device(device_number, device_id) {
						info!("irq_data: {:?}", irq_data);
						debug!(
							"Initialize interrupt pin {} and interrupt line {} for device 0x{:x}",
							irq_data.dev_pin, irq_data.irq_num, irq_data.dev_id
						);
						dev.set_irq(irq_data.dev_pin, irq_data.irq_num.try_into().unwrap());
					} else {
						info!("device id: 0x{:x} does not support interrupt", device_id);
					}

					/*
					if let Some((pin, line)) = detect_interrupt(
						bus.try_into().unwrap(),
						device_number.into(),
						&dtb,
						&parts,
					) {
						debug!(
							"Initialize interrupt pin {} and line {} for device {}",
							pin, line, device_id
						);
						dev.set_irq(pin, line);
					}
					*/

					unsafe {
						PCI_DEVICES.push(dev);
					}
				}

				return;
			} else if str::from_utf8(compatible)
				.unwrap()
				.contains("pci-host-cam-generic")
			{
				warn!("Currently, pci-host-cam-generic isn't supported!");
			}
		}
	}

	warn!("Unable to find PCI bus");
}
