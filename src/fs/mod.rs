use core::cell::{RefCell, RefMut};
use core::mem::{size_of, MaybeUninit};
use core::sync::atomic::{compiler_fence, Ordering};

use cstr_core::CStr;
use littlefs2::fs::{Allocation, File, FileAllocation, Filesystem, OpenOptions};
use littlefs2::io::SeekFrom;
use littlefs2::path::Path;

use crate::drivers::{pci, VirtioBlkDriver};
use crate::fd::FileDescriptor;
#[cfg(feature = "pci")]
pub mod fuse;

pub mod fatfs;

const FILE_TABLE_SIZE: usize = 128;

pub static mut ALLOC: MaybeUninit<Allocation<VirtioBlkDriver>> = MaybeUninit::uninit();
pub static mut BLK: MaybeUninit<VirtioBlkDriver> = MaybeUninit::uninit();
pub static mut FS: MaybeUninit<Filesystem<'static, VirtioBlkDriver>> = MaybeUninit::uninit();

pub static mut FILES: MaybeUninit<FileInfo> = MaybeUninit::uninit();

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
		FILES.write(FileInfo::new());
	}
}

const O_RDONLY: i32 = 0o0000;
const O_WRONLY: i32 = 0o0001;
const O_RDWR: i32 = 0o0002;
const O_CREAT: i32 = 0o0100;
const O_EXCL: i32 = 0o0200;
const O_TRUNC: i32 = 0o1000;
const O_APPEND: i32 = 0o2000;
const O_DIRECT: i32 = 0o40000;
const SEEK_SET: i32 = 0;
const SEEK_CUR: i32 = 1;
const SEEK_END: i32 = 2;

fn alloc_mut() -> &'static mut Allocation<VirtioBlkDriver> {
	unsafe { ALLOC.assume_init_mut() }
}
fn alloc_ref() -> &'static Allocation<VirtioBlkDriver> {
	unsafe { ALLOC.assume_init_ref() }
}

fn fs_mut() -> &'static mut Filesystem<'static, VirtioBlkDriver> {
	unsafe { FS.assume_init_mut() }
}

fn fs_ref() -> &'static Filesystem<'static, VirtioBlkDriver> {
	unsafe { FS.assume_init_ref() }
}
fn files_mut() -> &'static mut FileInfo {
	unsafe { FILES.assume_init_mut() }
}

fn files_ref() -> &'static FileInfo {
	unsafe { FILES.assume_init_ref() }
}
type FT = File<'static, 'static, VirtioBlkDriver>;
type FA = FileAllocation<VirtioBlkDriver>;
struct FileInfo {
	alloc: [MaybeUninit<RefCell<FA>>; FILE_TABLE_SIZE],
	files: [Option<FT>; FILE_TABLE_SIZE],
}

impl FileInfo {
	const INIT: Option<FT> = None;
	const FD_START: usize = 3; //0 1 2 for stdio

	fn new() -> Self {
		let mut f = FileInfo {
			alloc: MaybeUninit::uninit_array(),
			files: [Self::INIT; FILE_TABLE_SIZE],
		};
		for i in 0..FILE_TABLE_SIZE {
			f.alloc[i].write(RefCell::default());
		}

		f
	}

	fn alloc_fd(&'static mut self) -> Option<(FileDescriptor, &'static mut FA)> {
		unsafe {
			for fd in Self::FD_START..FILE_TABLE_SIZE {
				if self.files[fd as usize].is_none() {
					return Some((
						fd as FileDescriptor,
						self.alloc[fd as usize].assume_init_mut().get_mut(),
					));
				}
			}
		}
		None
	}

	fn set_file(&mut self, fd: FileDescriptor, file: FT) {
		let i = fd as usize;
		assert!(self.files[i].is_none());

		self.files[i] = Some(file);
	}

	fn close_file(&mut self, fd: FileDescriptor) {
		let i = fd as usize;
		assert!(self.files[i].is_some());

		unsafe {
			let f = self.files[i].take().unwrap();
			f.close().unwrap();

			let alloc: *mut FA = self.alloc[i].assume_init_mut().get_mut();
			alloc.write_bytes(0u8, size_of::<FA>());
		}
	}

	fn get_file(&self, fd: FileDescriptor) -> Option<&FT> {
		let i = fd as usize;
		self.files[i].as_ref()
	}
	fn get_file_mut(&mut self, fd: FileDescriptor) -> Option<&mut FT> {
		let i = fd as usize;
		self.files[i].as_mut()
	}
}

pub struct LittleFs {}

pub static LITTLEFS: LittleFs = LittleFs::new();

impl LittleFs {
	pub const fn new() -> Self {
		Self {}
	}
	fn mode_to_option(flags: i32, _mode: i32) -> OpenOptions {
		let mut opt = OpenOptions::new();

		//just always rw..
		opt.read(true).write(true);

		if flags & O_APPEND != 0 {
			opt.append(true);
		}

		if flags & O_CREAT != 0 {
			opt.create(true);
		}

		if flags & O_TRUNC != 0 {
			opt.truncate(true);
		}

		opt
	}
	pub fn open(&self, name: *const u8, flags: i32, mode: i32) -> Result<FileDescriptor, i32> {
		info!("enter open");
		unsafe {
			let (fd, alloc) = files_mut().alloc_fd().unwrap();
			let path = Path::from_cstr(CStr::from_ptr(name)).unwrap();

			let f = Self::mode_to_option(flags, mode)
				.open(fs_ref(), alloc, path)
				.map_err(|e| {
					info!("{:?}", e);
					-1i32
				})?;

			files_mut().set_file(fd, f);

			Ok(fd)
		}
	}

	pub fn read(&self, fd: FileDescriptor, buf: *mut u8, len: usize) -> isize {
		info!("read: {fd}, len: {len}");
		unsafe {
			let file = files_ref().get_file(fd).unwrap();

			let b = core::slice::from_raw_parts_mut(buf, len);
			let sz = file.read(b).unwrap();
			sz as isize
		}
	}

	pub fn write(&self, fd: FileDescriptor, buf: *const u8, len: usize) -> isize {
		info!("write: {fd}, len: {len}");
		unsafe {
			let file = files_ref().get_file(fd).unwrap();
			let b = core::slice::from_raw_parts(buf, len);
			let sz = file.write(b).unwrap();
			sz as isize
		}
	}

	pub fn lseek(&self, fd: FileDescriptor, offset: isize, whence: i32) -> isize {
		let seek = match whence {
			SEEK_SET => SeekFrom::Start(offset as u32),
			SEEK_END => SeekFrom::End(offset as i32),
			SEEK_CUR => SeekFrom::Current(offset as i32),
			_ => panic!("wrong whence"),
		};
		unsafe {
			let file = files_ref().get_file(fd).unwrap();
			file.seek(seek).map_or_else(|_| -1isize, |v| v as isize)
		}
	}

	pub fn close(&self, fd: FileDescriptor) -> i32 {
		files_mut().close_file(fd);
		0i32
	}
}
