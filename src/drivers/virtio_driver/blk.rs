use generic_array::typenum::{U512, U8};
use virtio_drivers::device::blk::VirtIOBlk;
use virtio_drivers::transport::pci::PciTransport;

use crate::drivers::pci::VirtioHal;

static EMPTY_BUFFER: [u8; 512] = [0xff; 512];

type VirtioBlk = VirtIOBlk<VirtioHal, PciTransport>;
pub struct VirtioBlkDriver {
	blk: VirtioBlk,
}

impl VirtioBlkDriver {
	pub fn new(blk: VirtioBlk) -> Self {
		Self { blk }
	}
}
