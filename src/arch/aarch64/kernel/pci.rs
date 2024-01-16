// doc: https://www.openfirmware.info/data/docs/bus.pci.pdf

use alloc::vec::Vec;
use core::convert::TryFrom;
use core::iter::Iterator;
use core::mem::size_of;
use core::{str, u32, u64, u8};

use arm_gic::gicv3::{IntId, Trigger};
use bit_field::BitField;
use hermit_dtb::Dtb;
use pci_types::{
	Bar, ConfigRegionAccess, InterruptLine, InterruptPin, PciAddress, PciHeader, VendorId, MAX_BARS,
};

use crate::arch::aarch64::kernel::interrupts::GIC;
use crate::arch::aarch64::mm::paging::{self, BasePageSize, PageSize, PageTableEntryFlags};
use crate::arch::aarch64::mm::{virtualmem, PhysAddr, VirtAddr};
use crate::drivers::pci::{PciBitfield, PciCommand, PciDevice, PCI_DEVICES};
use crate::kernel::boot_info;

const PCI_MAX_DEVICE_NUMBER: u8 = 32;
const PCI_MAX_FUNCTION_NUMBER: u8 = 8;

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
	type Item = PciAddress;

	fn next(&mut self) -> Option<Self::Item> {
		if self.end_of_iteration {
			return None;
		}

		//currently only single-function device is supported
		let probe_addr = PciAddress::new(0, self.bus, self.dev, 0);

		if self.bus == self.bus_end && self.dev == 0b11111 {
			self.end_of_iteration = true;
		} else if self.dev < 0b11111 {
			self.dev += 1;
		} else {
			self.bus += 1;
			self.dev = 0;
		}

		return Some(probe_addr);
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
#[allow(unused_assignments)]
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

		let (bitfield, child_address) = child_address.split_at(size_of::<u32>());
		info!(
			"child address: 0x{:x}",
			u64::from_be_bytes(child_address.try_into().unwrap())
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
#[allow(unused_assignments)]
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

				let (mut io_start, mem32_start, mut mem64_start) =
					detect_pci_regions2(&dtb, &parts);

				debug!("IO address space starts at{:#X}", io_start);
				debug!("Memory32 address space starts at {:#X}", mem32_start);
				debug!("Memory64 address space starts {:#X}", mem64_start);
				assert!(io_start > 0);
				assert!(mem32_start > 0);
				assert!(mem64_start > 0);

				let pci_config = PciConfigRegion::new(pci_address);

				//scan PCIE
				let pci_iter = PciConfigRegionIter::new(bus_end.try_into().unwrap());
				for pa in pci_iter {
					let pd = PciHeader::new(pa);
					let (vendor_id, device_id) = pd.id(&pci_config);
					let (header, has_multiple_functions) = (
						pd.header_type(&pci_config),
						pd.has_multiple_functions(&pci_config),
					);
					if device_id != 0xFFFF {
						info!("device id: 0x{:x}, vendor id: 0x{:x}, header: {:?}, has_multiple_functions: {}", device_id, vendor_id, header, has_multiple_functions);
					} else {
						continue;
					}
					let dev = PciDevice::new(pa, pci_config);
					// Initializes BARs
					let mut cmd = PciCommand::default();
					for i in 0..MAX_BARS {
						if let Some(bar) = dev.get_bar(i.try_into().unwrap()) {
							info!("Bar: {:?}", bar);
							match bar {
								Bar::Io { .. } => {
									dev.set_bar(
										i.try_into().unwrap(),
										Bar::Io {
											port: io_start.try_into().unwrap(),
										},
									);
									io_start += 0x20;
									cmd |=
										PciCommand::PCI_COMMAND_IO | PciCommand::PCI_COMMAND_MASTER;
								}
								// Currently, we ignore 32 bit memory bars
								/*Bar::Memory32 { address, size, prefetchable } => {
									dev.set_bar(i.try_into().unwrap(), Bar::Memory32 { address: mem32_start.try_into().unwrap(), size,  prefetchable });
									mem32_start += u64::from(size);
									cmd |= PciCommand::PCI_COMMAND_MEMORY|PciCommand::PCI_COMMAND_MASTER;
								}*/
								Bar::Memory64 {
									address: _,
									size,
									prefetchable,
								} => {
									dev.set_bar(
										i.try_into().unwrap(),
										Bar::Memory64 {
											address: mem64_start,
											size,
											prefetchable,
										},
									);
									mem64_start += size;
									cmd |= PciCommand::PCI_COMMAND_MEMORY
										| PciCommand::PCI_COMMAND_MASTER;
								}
								_ => {}
							}
						}
					}
					dev.set_command(cmd);

					let (bus, device) = (pa.bus(), pa.device());

					if let Some((pin, line)) =
						detect_interrupt(bus.try_into().unwrap(), device.into(), &dtb, &parts)
					{
						debug!(
							"Initialize interrupt pin {} and line {} for device {}",
							pin, line, device_id
						);
						dev.set_irq(pin, line);
					}

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
