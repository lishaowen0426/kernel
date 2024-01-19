#![allow(dead_code)]

use alloc::alloc::{alloc_zeroed, dealloc, handle_alloc_error};
use alloc::vec::Vec;
use core::alloc::Layout;
use core::convert::TryFrom;
use core::fmt;
use core::ptr::NonNull;

use aarch64::regs::MIDR_EL1::Revision;
use bitflags::bitflags;
use hermit_sync::without_interrupts;
#[cfg(any(feature = "tcp", feature = "udp", feature = "fs"))]
use hermit_sync::InterruptTicketMutex;
use pci_types::{
	Bar, BaseClass, ConfigRegionAccess, DeviceId, DeviceRevision, EndpointHeader, HeaderType,
	Interface, InterruptLine, InterruptPin, PciAddress, PciHeader, SubClass, VendorId, MAX_BARS,
};
use virtio_drivers::device::blk::VirtIOBlk;
use virtio_drivers::device::net::VirtIONetRaw;
use virtio_drivers::transport::pci::bus::{self, Cam, DeviceFunction, DeviceFunctionInfo, PciRoot};
use virtio_drivers::transport::pci::{virtio_device_type, PciTransport};
use virtio_drivers::transport::{DeviceType, Transport};
use virtio_drivers::{self, BufferDirection, PAGE_SIZE};

use crate::aarch64::mm::paging;
use crate::arch::mm::paging::{virtual_to_physical, BasePageSize, PageSize, PageTableEntryFlags};
use crate::arch::mm::{virtualmem, PhysAddr, VirtAddr};
use crate::arch::pci::{PciConfigRegion, PCI_ROOT_ADDR};
#[cfg(feature = "fs")]
use crate::drivers::fs::virtio_fs::VirtioFsDriver;
#[cfg(feature = "rtl8139")]
use crate::drivers::net::rtl8139::{self, RTL8139Driver};
#[cfg(all(not(feature = "rtl8139"), any(feature = "tcp", feature = "udp")))]
use crate::drivers::net::virtio_net::VirtioNetDriver;
#[cfg(any(
	all(any(feature = "tcp", feature = "udp"), not(feature = "rtl8139")),
	feature = "fs"
))]
use crate::drivers::virtio::transport::pci as pci_virtio;
#[cfg(any(
	all(any(feature = "tcp", feature = "udp"), not(feature = "rtl8139")),
	feature = "fs"
))]
use crate::drivers::virtio::transport::pci::VirtioDriver;
use crate::drivers::VirtioBlkDriver;

#[derive(Default, Debug, Copy, Clone)]
pub struct PciBitfield {
	relocatable: bool,
	prefetchable: bool,
	aliased: bool,
	ss: u8,
	bus: u8,
	dev: u8,
	function: u8,
	reg: u8,
}

impl TryFrom<&[u8]> for PciBitfield {
	type Error = error::PciError;

	fn try_from(value: &[u8]) -> Result<Self, Self::Error> {
		if value.len() != 4 {
			Err(error::PciError::General(0))
		} else if value[0] & 0b11100 != 0 {
			//safety check
			Err(error::PciError::General(0))
		} else {
			let relocatable = (value[0] & 0b10000000) == 0; // 0: relocatable
			let prefetchable = (value[0] & 0b01000000) != 0; //1: prefetchable
			let aliased = (value[0] & 0b00100000) != 0; //1: aliased
			let ss: u8 = value[0] & 0b11;
			let bus: u8 = value[1];
			let dev: u8 = value[2] >> 3;
			let function: u8 = value[2] & 0b111;
			let reg: u8 = value[3];
			Ok(Self {
				relocatable,
				prefetchable,
				aliased,
				ss,
				bus,
				dev,
				function,
				reg,
			})
		}
	}
}
/// Converts a given little endian coded u32 to native endian coded.
//
// INFO: As the endianness received from the device is little endian coded
// the given value must be swapped again on big endian machines. Which is done
// via the u32::to_le() method as the u32::to_be() would be a no-op in big endian
// machines. Resulting in no conversion.
#[inline]
pub(crate) fn from_pci_endian(val: u32) -> u32 {
	if cfg!(target = "big_endian") {
		val.to_le()
	} else {
		val
	}
}

/// The module contains constants specific to PCI.
#[allow(dead_code)]
pub(crate) mod constants {
	// PCI constants
	pub(crate) const PCI_MAX_BUS_NUMBER: u8 = 32;
	pub(crate) const PCI_MAX_DEVICE_NUMBER: u8 = 32;
	pub(crate) const PCI_CONFIG_ADDRESS_PORT: u16 = 0xCF8;
	pub(crate) const PCI_CONFIG_ADDRESS_ENABLE: u32 = 1 << 31;
	pub(crate) const PCI_CONFIG_DATA_PORT: u16 = 0xCFC;
	pub(crate) const PCI_CAP_ID_VNDR_VIRTIO: u32 = 0x09;
	pub(crate) const PCI_MASK_IS_DEV_BUS_MASTER: u32 = 0x0000_0004u32;
}

bitflags! {
	#[derive(Default, Clone, Copy, Debug, PartialEq, Eq, Hash)]
	pub struct PciCommand: u32 {
		/// Enable response in I/O space
		const PCI_COMMAND_IO = 0x1;
		/// Enable response in Memory space
		const PCI_COMMAND_MEMORY = 0x2;
		/// Enable bus mastering
		const PCI_COMMAND_MASTER = 0x4;
		/// Enable response to special cycles
		const PCI_COMMAND_SPECIAL = 0x8;
		// Use memory write and invalidate
		const PCI_COMMAND_INVALIDATE = 0x10;
		/// Enable palette snooping
		const PCI_COMMAND_VGA_PALETTE = 0x20;
		/// Enable parity checking
		const PCI_COMMAND_PARITY = 0x40;
		/// Enable address/data stepping
		const PCI_COMMAND_WAIT = 0x80;
		/// Enable SERR
		const PCI_COMMAND_SERR = 0x100;
		///  Device is allowed to generate fast back-to-back transactions;
		const PCI_COMMAND_FAST_BACK = 0x200;
		/// INTx# signal is disabled
		const PCI_COMMAND_INTX_DISABLE = 0x400;
	}
}

/// PCI registers offset inside header,
/// if PCI header is of type 00h (general device).
#[allow(dead_code, non_camel_case_types)]
#[repr(u16)]
pub enum DeviceHeader {
	PCI_ID_REGISTER = 0x00u16,
	PCI_COMMAND_REGISTER = 0x04u16,
	PCI_CLASS_REGISTER = 0x08u16,
	PCI_HEADER_REGISTER = 0x0Cu16,
	PCI_BAR0_REGISTER = 0x10u16,
	PCI_CAPABILITY_LIST_REGISTER = 0x34u16,
	PCI_INTERRUPT_REGISTER = 0x3Cu16,
}

impl From<DeviceHeader> for u16 {
	fn from(val: DeviceHeader) -> u16 {
		match val {
			DeviceHeader::PCI_ID_REGISTER => 0x00u16,
			DeviceHeader::PCI_COMMAND_REGISTER => 0x04u16,
			DeviceHeader::PCI_CLASS_REGISTER => 0x08u16,
			DeviceHeader::PCI_HEADER_REGISTER => 0x0Cu16,
			DeviceHeader::PCI_BAR0_REGISTER => 0x10u16,
			DeviceHeader::PCI_CAPABILITY_LIST_REGISTER => 0x34u16,
			DeviceHeader::PCI_INTERRUPT_REGISTER => 0x3Cu16,
		}
	}
}

/// PCI masks. For convenience put into an enum and provides
/// an `Into<u32>` method for usage.
#[allow(dead_code, non_camel_case_types)]
#[repr(u32)]
pub enum Masks {
	PCI_MASK_IS_BAR_IO_BAR = 0x0000_0001u32,
	PCI_MASK_IS_MEM_BASE_ADDRESS_64BIT = 0x0000_0004u32,
	PCI_MASK_IS_MEM_BAR_PREFETCHABLE = 0x0000_0008u32,
	PCI_MASK_STATUS_CAPABILITIES_LIST = 0x0000_0010u32,
	PCI_MASK_CAPLIST_POINTER = 0x0000_00FCu32,
	PCI_MASK_HEADER_TYPE = 0x007F_0000u32,
	PCI_MASK_MULTIFUNCTION = 0x0080_0000u32,
	PCI_MASK_MEM_BASE_ADDRESS = 0xFFFF_FFF0u32,
	PCI_MASK_IO_BASE_ADDRESS = 0xFFFF_FFFCu32,
}

impl From<Masks> for u32 {
	fn from(val: Masks) -> u32 {
		match val {
			Masks::PCI_MASK_STATUS_CAPABILITIES_LIST => 0x0000_0010u32,
			Masks::PCI_MASK_CAPLIST_POINTER => 0x0000_00FCu32,
			Masks::PCI_MASK_HEADER_TYPE => 0x007F_0000u32,
			Masks::PCI_MASK_MULTIFUNCTION => 0x0080_0000u32,
			Masks::PCI_MASK_MEM_BASE_ADDRESS => 0xFFFF_FFF0u32,
			Masks::PCI_MASK_IO_BASE_ADDRESS => 0xFFFF_FFFCu32,
			Masks::PCI_MASK_IS_MEM_BAR_PREFETCHABLE => 0x0000_0008u32,
			Masks::PCI_MASK_IS_MEM_BASE_ADDRESS_64BIT => 0x0000_0004u32,
			Masks::PCI_MASK_IS_BAR_IO_BAR => 0x0000_0001u32,
		}
	}
}

pub static mut PCI_DEVICES: Vec<PciDevice<PciConfigRegion>> = Vec::new();
static mut PCI_DRIVERS: Vec<PciDriver> = Vec::new();

#[derive(Copy, Clone, Debug)]
pub struct PciDevice<T: ConfigRegionAccess> {
	address: PciAddress,
	access: T,
}

impl<T: ConfigRegionAccess> PciDevice<T> {
	pub const fn new(address: PciAddress, access: T) -> Self {
		Self { address, access }
	}

	pub fn read_register(&self, register: u16) -> u32 {
		unsafe { self.access.read(self.address, register) }
	}

	pub fn write_register(&self, register: u16, value: u32) {
		unsafe { self.access.write(self.address, register, value) }
	}

	/// Set flag to the command register
	pub fn set_command(&self, cmd: PciCommand) {
		unsafe {
			let mut command = self
				.access
				.read(self.address, DeviceHeader::PCI_COMMAND_REGISTER.into());
			command |= cmd.bits();
			self.access.write(
				self.address,
				DeviceHeader::PCI_COMMAND_REGISTER.into(),
				command,
			)
		}
	}

	/// Get value of the command register
	pub fn get_command(&self) -> PciCommand {
		unsafe {
			PciCommand::from_bits(
				self.access
					.read(self.address, DeviceHeader::PCI_COMMAND_REGISTER.into()),
			)
			.unwrap()
		}
	}

	/// Returns the bar at bar-register `slot`.
	pub fn get_bar(&self, slot: u8) -> Option<Bar> {
		let header = PciHeader::new(self.address);
		if let Some(endpoint) = EndpointHeader::from_header(header, &self.access) {
			return endpoint.bar(slot, &self.access);
		}

		None
	}

	/// Configure the bar at register `slot`
	#[allow(unused_variables)]
	pub fn set_bar(&self, slot: u8, bar: Bar) {
		let cmd = u16::from(DeviceHeader::PCI_BAR0_REGISTER) + u16::from(slot) * 4;
		match bar {
			Bar::Io { port } => unsafe {
				self.access.write(self.address, cmd, port | 1);
			},
			Bar::Memory32 {
				address,
				size,
				prefetchable,
			} => {
				if prefetchable {
					unsafe {
						self.access.write(self.address, cmd, address | 1 << 3);
					}
				} else {
					unsafe {
						self.access.write(self.address, cmd, address);
					}
				}
			}
			Bar::Memory64 {
				address,
				size,
				prefetchable,
			} => {
				let high: u32 = (address >> 32).try_into().unwrap();
				let low: u32 = (address & 0xFFFF_FFF0u64).try_into().unwrap();
				info!(
					"address: 0x{:x}, high:0x{:x}, low:0x{:x}, prefetchable:{}",
					address, high, low, prefetchable
				);
				if prefetchable {
					unsafe {
						self.access.write(self.address, cmd, low | 2 << 1 | 1 << 3);
					}
				} else {
					unsafe {
						self.access.write(self.address, cmd, low | 2 << 1);
					}
				}
				unsafe {
					self.access.write(self.address, cmd + 4, high);
				}
			}
		}
	}

	/// Memory maps pci bar with specified index to identical location in virtual memory.
	/// no_cache determines if we set the `Cache Disable` flag in the page-table-entry.
	/// Returns (virtual-pointer, size) if successful, else None (if bar non-existent or IOSpace)
	pub fn memory_map_bar(&self, index: u8, no_cache: bool) -> Option<(VirtAddr, usize)> {
		let (address, size, prefetchable, width) = match self.get_bar(index) {
			Some(Bar::Io { .. }) => {
				warn!("Cannot map IOBar!");
				return None;
			}
			Some(Bar::Memory32 {
				address,
				size,
				prefetchable,
			}) => (
				u64::from(address),
				usize::try_from(size).unwrap(),
				prefetchable,
				32,
			),
			Some(Bar::Memory64 {
				address,
				size,
				prefetchable,
			}) => (address, usize::try_from(size).unwrap(), prefetchable, 64),
			_ => {
				return None;
			}
		};

		debug!(
			"Mapping bar {} at {:#x} with length {:#x}",
			index, address, size
		);

		if width != 64 {
			warn!("Currently only mapping of 64 bit bars is supported!");
			return None;
		}
		if !prefetchable {
			warn!("Currently only mapping of prefetchable bars is supported!")
		}

		// Since the bios/bootloader manages the physical address space, the address got from the bar is unique and not overlapping.
		// We therefore do not need to reserve any additional memory in our kernel.
		// Map bar into RW^X virtual memory
		let physical_address = address;
		let virtual_address = crate::mm::map(
			PhysAddr::from(physical_address),
			size,
			true,
			false,
			no_cache,
		);

		Some((virtual_address, size))
	}

	pub fn get_irq(&self) -> Option<InterruptLine> {
		let header = PciHeader::new(self.address);
		if let Some(endpoint) = EndpointHeader::from_header(header, &self.access) {
			let (_pin, line) = endpoint.interrupt(&self.access);
			Some(line)
		} else {
			None
		}
	}

	pub fn set_irq(&self, pin: InterruptPin, line: InterruptLine) {
		unsafe {
			let mut command = self
				.access
				.read(self.address, DeviceHeader::PCI_INTERRUPT_REGISTER.into());
			command &= 0xFFFF_0000u32;
			command |= u32::from(line);
			command |= u32::from(pin) << 8;
			self.access.write(
				self.address,
				DeviceHeader::PCI_INTERRUPT_REGISTER.into(),
				command,
			);
		}
	}

	pub fn bus(&self) -> u8 {
		self.address.bus()
	}

	pub fn device(&self) -> u8 {
		self.address.device()
	}

	pub fn vendor_id(&self) -> VendorId {
		let header = PciHeader::new(self.address);
		let (vendor_id, _device_id) = header.id(&self.access);
		vendor_id
	}

	pub fn device_id(&self) -> DeviceId {
		let header = PciHeader::new(self.address);
		let (_vendor_id, device_id) = header.id(&self.access);
		device_id
	}

	pub fn id(&self) -> (VendorId, DeviceId) {
		let header = PciHeader::new(self.address);
		header.id(&self.access)
	}

	pub fn revision_and_class(&self) -> (DeviceRevision, BaseClass, SubClass, Interface) {
		let header = PciHeader::new(self.address);
		header.revision_and_class(&self.access)
	}

	pub fn header_type(&self) -> HeaderType {
		let header = PciHeader::new(self.address);
		header.header_type(&self.access)
	}
}

impl<T: ConfigRegionAccess> fmt::Display for PciDevice<T> {
	fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
		let header = PciHeader::new(self.address);
		let header_type = header.header_type(&self.access);
		let (vendor_id, device_id) = header.id(&self.access);
		let (_dev_rev, class_id, subclass_id, _interface) = header.revision_and_class(&self.access);

		if let Some(endpoint) = EndpointHeader::from_header(header, &self.access) {
			#[cfg(feature = "pci-ids")]
			let (class_name, vendor_name, device_name) = {
				use pci_ids::{Class, Device, FromId, Subclass};

				let class_name = Class::from_id(class_id).map_or("Unknown Class", |class| {
					class
						.subclasses()
						.find(|s| s.id() == subclass_id)
						.map(Subclass::name)
						.unwrap_or_else(|| class.name())
				});

				let (vendor_name, device_name) = Device::from_vid_pid(vendor_id, device_id)
					.map(|device| (device.vendor().name(), device.name()))
					.unwrap_or(("Unknown Vendor", "Unknown Device"));

				(class_name, vendor_name, device_name)
			};

			#[cfg(not(feature = "pci-ids"))]
			let (class_name, vendor_name, device_name) = ("Unknown Class", "Unknown Vendor", "Unknown Device");

			// Output detailed readable information about this device.
			write!(
				f,
				"{:02X}:{:02X} {} [{:02X}{:02X}]: {} {} [{:04X}:{:04X}]",
				self.address.bus(),
				self.address.device(),
				class_name,
				class_id,
				subclass_id,
				vendor_name,
				device_name,
				vendor_id,
				device_id
			)?;

			// If the devices uses an IRQ, output this one as well.
			let (pin, irq) = endpoint.interrupt(&self.access);
			write!(f, "irq pin: {}, irq line:{}", pin, irq);
			if irq != 0 && irq != u8::MAX {
				write!(f, ", IRQ {irq}")?;
			}

			let mut slot: u8 = 0;
			while usize::from(slot) < MAX_BARS {
				if let Some(pci_bar) = endpoint.bar(slot, &self.access) {
					match pci_bar {
						Bar::Memory64 {
							address,
							size,
							prefetchable,
						} => {
							write!(f, ", BAR{slot} Memory64 {{ address: {address:#X}, size: {size:#X}, prefetchable: {prefetchable} }}")?;
							slot += 1;
						}
						Bar::Memory32 {
							address,
							size,
							prefetchable,
						} => {
							write!(f, ", BAR{slot} Memory32 {{ address: {address:#X}, size: {size:#X}, prefetchable: {prefetchable} }}")?;
						}
						Bar::Io { port } => {
							write!(f, ", BAR{slot} IO {{ port: {port:#X} }}")?;
						}
					}
				}
				slot += 1;
			}
		} else {
			// Output detailed readable information about this device.
			write!(
				f,
				"{:02X}:{:02X} {:?} [{:04X}:{:04X}]",
				self.address.bus(),
				self.address.device(),
				header_type,
				vendor_id,
				device_id
			)?;
		}

		Ok(())
	}
}

pub(crate) fn print_information() {
	infoheader!(" PCI BUS INFORMATION ");

	for adapter in unsafe { PCI_DEVICES.iter() } {
		info!("{}", adapter);
	}

	infofooter!();
}

#[allow(clippy::large_enum_variant)]
pub(crate) enum PciDriver {
	#[cfg(feature = "fs")]
	VirtioFs(InterruptTicketMutex<VirtioFsDriver>),
	#[cfg(all(not(feature = "rtl8139"), any(feature = "tcp", feature = "udp")))]
	VirtioNet(InterruptTicketMutex<VirtioNetDriver>),
	#[cfg(all(feature = "rtl8139", any(feature = "tcp", feature = "udp")))]
	RTL8139Net(InterruptTicketMutex<RTL8139Driver>),

	VirtioBlk(InterruptTicketMutex<VirtioBlkDriver>),
}

impl PciDriver {
	#[cfg(all(not(feature = "rtl8139"), any(feature = "tcp", feature = "udp")))]
	fn get_network_driver(&self) -> Option<&InterruptTicketMutex<VirtioNetDriver>> {
		#[allow(unreachable_patterns)]
		match self {
			Self::VirtioNet(drv) => Some(drv),
			_ => None,
		}
	}

	#[cfg(all(feature = "rtl8139", any(feature = "tcp", feature = "udp")))]
	fn get_network_driver(&self) -> Option<&InterruptTicketMutex<RTL8139Driver>> {
		#[allow(unreachable_patterns)]
		match self {
			Self::RTL8139Net(drv) => Some(drv),
			_ => None,
		}
	}

	#[cfg(feature = "fs")]
	fn get_filesystem_driver(&self) -> Option<&InterruptTicketMutex<VirtioFsDriver>> {
		match self {
			Self::VirtioFs(drv) => Some(drv),
			#[allow(unreachable_patterns)]
			_ => None,
		}
	}

	fn get_blk_driver(&self) -> Option<&InterruptTicketMutex<VirtioBlkDriver>> {
		match self {
			Self::VirtioBlk(drv) => Some(drv),
			#[allow(unreachable_patterns)]
			_ => None,
		}
	}
}

pub(crate) fn register_driver(drv: PciDriver) {
	unsafe {
		PCI_DRIVERS.push(drv);
	}
}

#[cfg(all(not(feature = "rtl8139"), any(feature = "tcp", feature = "udp")))]
pub(crate) fn get_network_driver() -> Option<&'static InterruptTicketMutex<VirtioNetDriver>> {
	unsafe { PCI_DRIVERS.iter().find_map(|drv| drv.get_network_driver()) }
}

#[cfg(all(feature = "rtl8139", any(feature = "tcp", feature = "udp")))]
pub(crate) fn get_network_driver() -> Option<&'static InterruptTicketMutex<RTL8139Driver>> {
	unsafe { PCI_DRIVERS.iter().find_map(|drv| drv.get_network_driver()) }
}

#[cfg(feature = "fs")]
pub(crate) fn get_filesystem_driver() -> Option<&'static InterruptTicketMutex<VirtioFsDriver>> {
	unsafe {
		PCI_DRIVERS
			.iter()
			.find_map(|drv| drv.get_filesystem_driver())
	}
}

pub(crate) fn get_blk_driver() -> Option<&'static InterruptTicketMutex<VirtioBlkDriver>> {
	unsafe { PCI_DRIVERS.iter().find_map(|drv| drv.get_blk_driver()) }
}
pub struct VirtioHal;
unsafe impl virtio_drivers::Hal for VirtioHal {
	fn dma_alloc(
		pages: usize,
		direction: BufferDirection,
	) -> (virtio_drivers::PhysAddr, NonNull<u8>) {
		let layout =
			Layout::from_size_align(pages * virtio_drivers::PAGE_SIZE, virtio_drivers::PAGE_SIZE)
				.unwrap();
		// Safe because the layout has a non-zero size.
		let vaddr = unsafe { alloc_zeroed(layout) };
		let vaddr = if let Some(vaddr) = NonNull::new(vaddr) {
			vaddr
		} else {
			handle_alloc_error(layout)
		};
		let paddr = virtual_to_physical(VirtAddr::from_u64(vaddr.as_ptr() as u64)).unwrap();
		(paddr.as_usize(), vaddr)
	}

	unsafe fn dma_dealloc(
		paddr: virtio_drivers::PhysAddr,
		vaddr: NonNull<u8>,
		pages: usize,
	) -> i32 {
		let layout = Layout::from_size_align(pages * PAGE_SIZE, PAGE_SIZE).unwrap();
		// Safe because the memory was allocated by `dma_alloc` above using the same allocator, and
		// the layout is the same as was used then.
		unsafe {
			dealloc(vaddr.as_ptr(), layout);
		}
		0
	}

	unsafe fn mmio_phys_to_virt(paddr: virtio_drivers::PhysAddr, size: usize) -> NonNull<u8> {
		let align_up =
			|value: usize, alignment: usize| -> usize { ((value - 1) | (alignment - 1)) + 1 };
		let page_count =
			align_up(size, BasePageSize::SIZE as usize) / (BasePageSize::SIZE as usize);
		let vaddr = virtualmem::allocate_aligned(size, size).unwrap();
		let mut flags = PageTableEntryFlags::empty();
		flags.device().writable().execute_disable();
		paging::map::<BasePageSize>(
			vaddr,
			PhysAddr::from(paddr),
			page_count.try_into().unwrap(),
			flags,
		);
		NonNull::new(vaddr.as_mut_ptr::<u8>()).unwrap()
	}

	unsafe fn share(buffer: NonNull<[u8]>, direction: BufferDirection) -> virtio_drivers::PhysAddr {
		let vaddr = VirtAddr::from_u64(buffer.as_ptr() as *mut u8 as u64);
		let paddr = paging::get_physical_address::<BasePageSize>(vaddr).unwrap();
		paddr.as_usize()
	}

	unsafe fn unshare(
		_paddr: virtio_drivers::PhysAddr,
		_buffer: NonNull<[u8]>,
		_direction: BufferDirection,
	) {
	}
}

fn virtio_net<T: Transport>(transport: T) {
	let mut net =
		VirtIONetRaw::<VirtioHal, T, 16>::new(transport).expect("failed to create net driver");
	let mut buf = [0u8; 2048];
	let (hdr_len, pkt_len) = net.receive_wait(&mut buf).expect("failed to recv");
	info!(
		"recv {} bytes: {:02x?}",
		pkt_len,
		&buf[hdr_len..hdr_len + pkt_len]
	);
	net.send(&buf[..hdr_len + pkt_len]).expect("failed to send");
	info!("virtio-net test finished");
}

fn virtio_blk(transport: PciTransport) {
	let blk =
		VirtIOBlk::<VirtioHal, PciTransport>::new(transport).expect("failed to create blk driver");
	assert!(!blk.readonly());
	assert!(blk.capacity() == 262144); //hardcode this number in the implementation of littlefs2 Storage

	register_driver(PciDriver::VirtioBlk(InterruptTicketMutex::new(
		VirtioBlkDriver::new(blk),
	)));
	info!("virtio-blk registered");
}

fn virtio_device(transport: PciTransport) {
	match transport.device_type() {
		DeviceType::Block => virtio_blk(transport),
		t => warn!("Unrecognized virtio device: {:?}", t),
	}
}
pub(crate) fn init_virtio_drivers() {
	unsafe {
		for dev in PCI_DEVICES.iter() {
			let (revision, class, subclass, prog_if) = dev.revision_and_class();
			let header_type: bus::HeaderType = match dev.header_type() {
				HeaderType::Endpoint => bus::HeaderType::Standard,
				HeaderType::CardBusBridge => bus::HeaderType::PciCardbusBridge,
				HeaderType::PciPciBridge => bus::HeaderType::PciPciBridge,
				HeaderType::Unknown(i) => bus::HeaderType::Unrecognised(i),
				_ => panic!("invalid pci device header type"),
			};
			let dev_info = DeviceFunctionInfo {
				vendor_id: dev.vendor_id(),
				device_id: dev.device_id(),
				class,
				subclass,
				prog_if,
				revision,
				header_type,
			};
			if let Some(virtio_type) = virtio_device_type(&dev_info) {
				info!("  VirtIO {:?}", virtio_type);
				//allocate_bars(&mut pci_root, device_function, &mut allocator);
				//dump_bar_contents(&mut pci_root, device_function, 4);
				let mut pci_root = PciRoot::new(PCI_ROOT_ADDR.as_mut_ptr(), Cam::Ecam);
				let mut transport = PciTransport::new::<VirtioHal>(
					&mut pci_root,
					DeviceFunction {
						bus: dev.bus(),
						device: dev.device(),
						function: 0,
					},
				)
				.unwrap();
				info!(
					"Detected virtio PCI device with device type {:?}, features {:#018x}",
					transport.device_type(),
					transport.read_device_features(),
				);
				virtio_device(transport);
			}
		}
	}
}

pub(crate) fn init_drivers() {
	// virtio: 4.1.2 PCI Device Discovery
	without_interrupts(|| {
		/*
		for adapter in unsafe {
			PCI_DEVICES.iter().filter(|x| {
				let (vendor_id, device_id) = x.id();
				vendor_id == 0x1AF4 && (0x1000..=0x107F).contains(&device_id)
			})
		} {
			info!(
				"Found virtio  device with device id {:#x}",
				adapter.device_id()
			);

			#[cfg(any(
				all(any(feature = "tcp", feature = "udp"), not(feature = "rtl8139")),
				feature = "fs"
			))]
			match pci_virtio::init_device(adapter) {
				#[cfg(all(not(feature = "rtl8139"), any(feature = "tcp", feature = "udp")))]
				Ok(VirtioDriver::Network(drv)) => {
					register_driver(PciDriver::VirtioNet(InterruptTicketMutex::new(drv)))
				}
				#[cfg(feature = "fs")]
				Ok(VirtioDriver::FileSystem(drv)) => {
					register_driver(PciDriver::VirtioFs(InterruptTicketMutex::new(drv)))
				}
				_ => {}
			}
		}
		*/
		init_virtio_drivers();

		// Searching for Realtek RTL8139, which is supported by Qemu
		#[cfg(feature = "rtl8139")]
		for adapter in unsafe {
			PCI_DEVICES.iter().filter(|x| {
				let (vendor_id, device_id) = x.id();
				vendor_id == 0x10ec && (0x8138..=0x8139).contains(&device_id)
			})
		} {
			info!(
				"Found Realtek network device with device id {:#x}",
				adapter.device_id()
			);

			if let Ok(drv) = rtl8139::init_device(adapter) {
				register_driver(PciDriver::RTL8139Net(InterruptTicketMutex::new(drv)))
			}
		}
	});
}

/// A module containing PCI specific errors
///
/// Errors include...
pub(crate) mod error {
	/// An enum of PciErrors
	/// typically carrying the device's id as an u16.
	#[derive(Debug)]
	pub enum PciError {
		General(u16),
		NoBar(u16),
		NoCapPtr(u16),
		BadCapPtr(u16),
		NoVirtioCaps(u16),
	}
}
