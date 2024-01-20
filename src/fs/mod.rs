use core::mem::MaybeUninit;
use core::sync::atomic::{compiler_fence, Ordering};

use littlefs2::fs::{Allocation, Filesystem};

use crate::drivers::{pci, VirtioBlkDriver};
#[cfg(feature = "pci")]
pub mod fuse;

pub mod fatfs;

pub static mut ALLOC: MaybeUninit<Allocation<VirtioBlkDriver>> = MaybeUninit::uninit();
pub static mut BLK: MaybeUninit<VirtioBlkDriver> = MaybeUninit::uninit();
pub static mut FS: MaybeUninit<Filesystem<'static, VirtioBlkDriver>> = MaybeUninit::uninit();

pub fn init() {
	#[cfg(feature = "pci")]
	fuse::init();
}

pub fn register_filesystem(mut blk: VirtioBlkDriver) {
	Filesystem::format(&mut blk).unwrap();
	let alloc: Allocation<VirtioBlkDriver> = Filesystem::allocate();
	unsafe {
		BLK.write(blk);
		ALLOC.write(alloc);
	}
	compiler_fence(Ordering::Release);
	unsafe {
		FS.write(Filesystem::mount(ALLOC.assume_init_mut(), BLK.assume_init_mut()).unwrap());
	}
}
