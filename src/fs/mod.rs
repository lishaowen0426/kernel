use crate::drivers::pci;
#[cfg(feature = "pci")]
pub mod fuse;

pub fn init() {
	#[cfg(feature = "pci")]
	fuse::init();

	let mut drv = pci::get_blk_driver().unwrap().lock();
}
