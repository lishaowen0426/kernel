#![allow(clippy::result_unit_err)]

#[cfg(feature = "newlib")]
use hermit_sync::InterruptTicketMutex;
use hermit_sync::Lazy;

pub use self::condvar::*;
pub use self::entropy::*;
pub use self::futex::*;
pub use self::processor::*;
#[cfg(feature = "newlib")]
pub use self::recmutex::*;
pub use self::semaphore::*;
pub use self::spinlock::*;
pub use self::system::*;
pub use self::tasks::*;
pub use self::timer::*;
use crate::env;
use crate::fd::{dup_object, get_object, remove_object, DirectoryEntry, FileDescriptor};
use crate::fs::LITTLEFS;
use crate::syscalls::fs::FileAttr;
use crate::syscalls::interfaces::SyscallInterface;
#[cfg(target_os = "none")]
use crate::{__sys_free, __sys_malloc, __sys_realloc};

mod condvar;
mod entropy;
pub(crate) mod fs;
mod futex;
mod interfaces;
#[cfg(feature = "newlib")]
mod lwip;
#[cfg(all(any(feature = "tcp", feature = "udp"), not(feature = "newlib")))]
pub mod net;
mod processor;
#[cfg(feature = "newlib")]
mod recmutex;
mod semaphore;
mod spinlock;
mod system;
mod tasks;
mod timer;

#[cfg(feature = "newlib")]
const LWIP_FD_BIT: i32 = 1 << 30;

#[cfg(feature = "newlib")]
pub static LWIP_LOCK: InterruptTicketMutex<()> = InterruptTicketMutex::new(());

pub(crate) static SYS: Lazy<&'static dyn SyscallInterface> = Lazy::new(|| {
	if env::is_uhyve() {
		&self::interfaces::Uhyve
	} else {
		&self::interfaces::Generic
	}
});

pub(crate) fn init() {
	Lazy::force(&SYS);

	// Perform interface-specific initialization steps.
	SYS.init();

	init_entropy();
	#[cfg(feature = "newlib")]
	sbrk_init();
}

#[cfg(target_os = "none")]
#[no_mangle]
pub extern "C" fn sys_malloc(size: usize, align: usize) -> *mut u8 {
	kernel_function!(__sys_malloc(size, align))
}

#[cfg(target_os = "none")]
#[no_mangle]
pub extern "C" fn sys_realloc(ptr: *mut u8, size: usize, align: usize, new_size: usize) -> *mut u8 {
	kernel_function!(__sys_realloc(ptr, size, align, new_size))
}

#[cfg(target_os = "none")]
#[no_mangle]
pub extern "C" fn sys_free(ptr: *mut u8, size: usize, align: usize) {
	kernel_function!(__sys_free(ptr, size, align))
}

pub(crate) fn get_application_parameters() -> (i32, *const *const u8, *const *const u8) {
	SYS.get_application_parameters()
}

pub(crate) extern "C" fn __sys_shutdown(arg: i32) -> ! {
	// print some performance statistics
	crate::arch::kernel::print_statistics();

	SYS.shutdown(arg)
}

#[no_mangle]
pub extern "C" fn sys_shutdown(arg: i32) -> ! {
	kernel_function!(__sys_shutdown(arg))
}

extern "C" fn __sys_unlink(name: *const u8) -> i32 {
	SYS.unlink(name)
}

#[no_mangle]
pub extern "C" fn sys_unlink(name: *const u8) -> i32 {
	kernel_function!(__sys_unlink(name))
}

extern "C" fn __sys_set_permission(name: *const u8, perm: u32) -> i32 {
	SYS.set_permission(name, perm)
}

#[no_mangle]
pub extern "C" fn sys_set_permission(name: *const u8, perm: u32) -> i32 {
	kernel_function!(__sys_set_permission(name, perm))
}

#[cfg(target_arch = "x86_64")]
extern "C" fn __sys_mkdir(name: *const u8, mode: u32) -> i32 {
	SYS.mkdir(name, mode)
}

#[cfg(not(target_arch = "x86_64"))]
extern "C" fn __sys_mkdir(_name: *const u8, _mode: u32) -> i32 {
	-crate::errno::ENOSYS
}

#[no_mangle]
pub extern "C" fn sys_mkdir(name: *const u8, mode: u32) -> i32 {
	kernel_function!(__sys_mkdir(name, mode))
}

#[cfg(target_arch = "x86_64")]
extern "C" fn __sys_rmdir(name: *const u8) -> i32 {
	SYS.rmdir(name)
}

#[cfg(not(target_arch = "x86_64"))]
extern "C" fn __sys_rmdir(_name: *const u8) -> i32 {
	-crate::errno::ENOSYS
}

#[no_mangle]
pub extern "C" fn sys_rmdir(name: *const u8) -> i32 {
	kernel_function!(__sys_rmdir(name))
}

extern "C" fn __sys_stat(name: *const u8, stat: *mut FileAttr) -> i32 {
	SYS.stat(name, stat)
}

#[no_mangle]
pub extern "C" fn sys_stat(name: *const u8, stat: *mut FileAttr) -> i32 {
	kernel_function!(__sys_stat(name, stat))
}

extern "C" fn __sys_lstat(name: *const u8, stat: *mut FileAttr) -> i32 {
	SYS.lstat(name, stat)
}

#[no_mangle]
pub extern "C" fn sys_lstat(name: *const u8, stat: *mut FileAttr) -> i32 {
	kernel_function!(__sys_lstat(name, stat))
}

extern "C" fn __sys_fstat(fd: FileDescriptor, stat: *mut FileAttr) -> i32 {
	let obj = get_object(fd);
	obj.map_or_else(|e| e, |v| (*v).fstat(stat))
}

#[no_mangle]
pub extern "C" fn sys_fstat(fd: FileDescriptor, stat: *mut FileAttr) -> i32 {
	kernel_function!(__sys_fstat(fd, stat))
}

extern "C" fn __sys_opendir(name: *const u8) -> FileDescriptor {
	crate::fd::opendir(name).map_or_else(|e| e, |v| v)
}

#[no_mangle]
pub extern "C" fn sys_opendir(name: *const u8) -> FileDescriptor {
	kernel_function!(__sys_opendir(name))
}

extern "C" fn __sys_open(name: *const u8, flags: i32, mode: i32) -> FileDescriptor {
	crate::fd::open(name, flags, mode).map_or_else(|e| e, |v| v)
}

#[no_mangle]
pub extern "C" fn sys_open(name: *const u8, flags: i32, mode: i32) -> FileDescriptor {
	//kernel_function!(__sys_open(name, flags, mode))
	match LITTLEFS.open(name, flags, mode) {
		Ok(fd) => fd,
		Err(e) => e,
	}
}

extern "C" fn __sys_close(fd: FileDescriptor) -> i32 {
	let obj = remove_object(fd);
	obj.map_or_else(|e| e, |_| 0)
}

#[no_mangle]
pub extern "C" fn sys_close(fd: FileDescriptor) -> i32 {
	//kernel_function!(__sys_close(fd))
	LITTLEFS.close(fd)
}

extern "C" fn __sys_read(fd: FileDescriptor, buf: *mut u8, len: usize) -> isize {
	let obj = get_object(fd);
	obj.map_or_else(|e| e as isize, |v| (*v).read(buf, len))
}

#[no_mangle]
pub extern "C" fn sys_read(fd: FileDescriptor, buf: *mut u8, len: usize) -> isize {
	if fd == 0 || fd == 1 || fd == 2 {
		kernel_function!(__sys_read(fd, buf, len))
	} else {
		LITTLEFS.read(fd, buf, len)
	}
}

extern "C" fn __sys_write(fd: FileDescriptor, buf: *const u8, len: usize) -> isize {
	let obj = get_object(fd);
	obj.map_or_else(|e| e as isize, |v| (*v).write(buf, len))
}

#[no_mangle]
pub extern "C" fn sys_write(fd: FileDescriptor, buf: *const u8, len: usize) -> isize {
	if fd == 0 || fd == 1 || fd == 2 {
		kernel_function!(__sys_write(fd, buf, len))
	} else {
		LITTLEFS.write(fd, buf, len)
	}
}

extern "C" fn __sys_ioctl(fd: FileDescriptor, cmd: i32, argp: *mut core::ffi::c_void) -> i32 {
	let obj = get_object(fd);
	obj.map_or_else(|e| e, |v| (*v).ioctl(cmd, argp))
}

#[no_mangle]
pub extern "C" fn sys_ioctl(fd: FileDescriptor, cmd: i32, argp: *mut core::ffi::c_void) -> i32 {
	kernel_function!(__sys_ioctl(fd, cmd, argp))
}

extern "C" fn __sys_lseek(fd: FileDescriptor, offset: isize, whence: i32) -> isize {
	let obj = get_object(fd);
	obj.map_or_else(
		|e| e as isize,
		|v| (*v).lseek(offset, num::FromPrimitive::from_i32(whence).unwrap()),
	)
}

#[no_mangle]
pub extern "C" fn sys_lseek(fd: FileDescriptor, offset: isize, whence: i32) -> isize {
	if fd == 0 || fd == 1 || fd == 2 {
		kernel_function!(__sys_lseek(fd, offset, whence))
	} else {
		LITTLEFS.lseek(fd, offset, whence)
	}
}

extern "C" fn __sys_readdir(fd: FileDescriptor) -> DirectoryEntry {
	let obj = get_object(fd);
	obj.map_or(DirectoryEntry::Invalid(-crate::errno::EINVAL), |v| {
		(*v).readdir()
	})
}

#[no_mangle]
pub extern "C" fn sys_readdir(fd: FileDescriptor) -> DirectoryEntry {
	kernel_function!(__sys_readdir(fd))
}

extern "C" fn __sys_dup(fd: i32) -> i32 {
	dup_object(fd).map_or_else(|e| e, |v| v)
}

#[no_mangle]
pub extern "C" fn sys_dup(fd: i32) -> i32 {
	kernel_function!(__sys_dup(fd))
}

extern "C" fn __sys_image_start_addr() -> usize {
	crate::mm::kernel_start_address().0.try_into().unwrap()
}

#[no_mangle]
pub extern "C" fn sys_image_start_addr() -> usize {
	kernel_function!(__sys_image_start_addr())
}
