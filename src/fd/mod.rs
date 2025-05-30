use alloc::sync::Arc;
use core::ffi::{c_void, CStr};
#[cfg(any(target_arch = "x86_64", target_arch = "aarch64"))]
use core::ptr;
use core::sync::atomic::{AtomicI32, Ordering};

use ahash::RandomState;
use dyn_clone::DynClone;
use hashbrown::HashMap;
#[cfg(target_arch = "x86_64")]
use x86::io::*;

use crate::arch::mm::{paging, PhysAddr, VirtAddr};
use crate::env;
use crate::errno::*;
use crate::fd::file::{GenericFile, UhyveFile};
use crate::fd::stdio::*;
use crate::syscalls::fs::{self, Dirent, FileAttr, FilePerms, SeekWhence};
#[cfg(all(any(feature = "tcp", feature = "udp"), not(feature = "newlib")))]
use crate::syscalls::net::*;

mod file;
#[cfg(all(any(feature = "tcp", feature = "udp"), not(feature = "newlib")))]
pub mod socket;
mod stdio;

const UHYVE_PORT_WRITE: u16 = 0x400;
const UHYVE_PORT_OPEN: u16 = 0x440;
const UHYVE_PORT_CLOSE: u16 = 0x480;
const UHYVE_PORT_READ: u16 = 0x500;
const UHYVE_PORT_LSEEK: u16 = 0x580;

const STDIN_FILENO: FileDescriptor = 0;
const STDOUT_FILENO: FileDescriptor = 1;
const STDERR_FILENO: FileDescriptor = 2;

pub(crate) type FileDescriptor = i32;

/// Mapping between file descriptor and the referenced object
static OBJECT_MAP: pflock::PFLock<HashMap<FileDescriptor, Arc<dyn ObjectInterface>, RandomState>> =
	pflock::PFLock::new(HashMap::<
		FileDescriptor,
		Arc<dyn ObjectInterface>,
		RandomState,
	>::with_hasher(RandomState::with_seeds(0, 0, 0, 0)));
/// Atomic counter to determine the next unused file descriptor
static FD_COUNTER: AtomicI32 = AtomicI32::new(3);

// TODO: these are defined in hermit-abi. Should we use a constants crate imported in both?
//const O_RDONLY: i32 = 0o0000;
const O_WRONLY: i32 = 0o0001;
const O_RDWR: i32 = 0o0002;
const O_CREAT: i32 = 0o0100;
const O_EXCL: i32 = 0o0200;
const O_TRUNC: i32 = 0o1000;
const O_APPEND: i32 = 0o2000;
const O_DIRECT: i32 = 0o40000;

#[repr(C, packed)]
struct SysOpen {
	name: PhysAddr,
	flags: i32,
	mode: i32,
	ret: i32,
}

impl SysOpen {
	fn new(name: VirtAddr, flags: i32, mode: i32) -> SysOpen {
		SysOpen {
			name: paging::virtual_to_physical(name).unwrap(),
			flags,
			mode,
			ret: -1,
		}
	}
}

#[repr(C, packed)]
struct SysClose {
	fd: i32,
	ret: i32,
}

impl SysClose {
	fn new(fd: i32) -> SysClose {
		SysClose { fd, ret: -1 }
	}
}

#[repr(C, packed)]
struct SysRead {
	fd: i32,
	buf: *const u8,
	len: usize,
	ret: isize,
}

impl SysRead {
	fn new(fd: i32, buf: *const u8, len: usize) -> SysRead {
		SysRead {
			fd,
			buf,
			len,
			ret: -1,
		}
	}
}

#[repr(C, packed)]
struct SysWrite {
	fd: i32,
	buf: *const u8,
	len: usize,
}

impl SysWrite {
	pub fn new(fd: i32, buf: *const u8, len: usize) -> SysWrite {
		SysWrite { fd, buf, len }
	}
}

#[repr(C, packed)]
struct SysLseek {
	pub fd: i32,
	pub offset: isize,
	pub whence: i32,
}

impl SysLseek {
	fn new(fd: i32, offset: isize, whence: SeekWhence) -> SysLseek {
		let whence: i32 = num::ToPrimitive::to_i32(&whence).unwrap();

		SysLseek { fd, offset, whence }
	}
}

/// forward a request to the hypervisor uhyve
#[inline]
#[cfg(target_arch = "x86_64")]
fn uhyve_send<T>(port: u16, data: &mut T) {
	let ptr = VirtAddr(ptr::from_mut(data).addr() as u64);
	let physical_address = paging::virtual_to_physical(ptr).unwrap();

	unsafe {
		outl(port, physical_address.as_u64() as u32);
	}
}

/// forward a request to the hypervisor uhyve
#[inline]
#[cfg(target_arch = "aarch64")]
fn uhyve_send<T>(port: u16, data: &mut T) {
	use core::arch::asm;

	let ptr = VirtAddr(ptr::from_mut(data).addr() as u64);
	let physical_address = paging::virtual_to_physical(ptr).unwrap();

	unsafe {
		asm!(
			"str x8, [{port}]",
			port = in(reg) u64::from(port),
			in("x8") physical_address.as_u64(),
			options(nostack),
		);
	}
}

/// forward a request to the hypervisor uhyve
#[inline]
#[cfg(target_arch = "riscv64")]
fn uhyve_send<T>(_port: u16, _data: &mut T) {
	todo!()
}

fn open_flags_to_perm(flags: i32, mode: u32) -> FilePerms {
	let mut perms = FilePerms {
		raw: flags as u32,
		mode,
		..Default::default()
	};
	perms.write = flags & (O_WRONLY | O_RDWR) != 0;
	perms.creat = flags & (O_CREAT) != 0;
	perms.excl = flags & (O_EXCL) != 0;
	perms.trunc = flags & (O_TRUNC) != 0;
	perms.append = flags & (O_APPEND) != 0;
	perms.directio = flags & (O_DIRECT) != 0;
	if flags & !(O_WRONLY | O_RDWR | O_CREAT | O_EXCL | O_TRUNC | O_APPEND | O_DIRECT) != 0 {
		warn!("Unknown file flags used! {}", flags);
	}
	perms
}

#[derive(Copy, Clone, Debug)]
#[repr(C)]
pub enum DirectoryEntry {
	Invalid(i32),
	Valid(*const Dirent),
}

pub trait ObjectInterface: Sync + Send + core::fmt::Debug + DynClone {
	/// `read` attempts to read `len` bytes from the object references
	/// by the descriptor
	fn read(&self, _buf: *mut u8, _len: usize) -> isize {
		(-ENOSYS).try_into().unwrap()
	}

	/// `write` attempts to write `len` bytes to the object references
	/// by the descriptor
	fn write(&self, _buf: *const u8, _len: usize) -> isize {
		(-EINVAL).try_into().unwrap()
	}

	/// `lseek` function repositions the offset of the file descriptor fildes
	fn lseek(&self, _offset: isize, _whence: SeekWhence) -> isize {
		(-EINVAL).try_into().unwrap()
	}

	/// `fstat`
	fn fstat(&self, _stat: *mut FileAttr) -> i32 {
		-EINVAL
	}

	/// `unlink` removes file entry
	fn unlink(&self, _name: *const u8) -> i32 {
		-EINVAL
	}

	/// `rmdir` removes directory entry
	fn rmdir(&self, _name: *const u8) -> i32 {
		-EINVAL
	}

	/// 'readdir' returns a pointer to a dirent structure
	/// representing the next directory entry in the directory stream
	/// pointed to by the file descriptor
	fn readdir(&self) -> DirectoryEntry {
		DirectoryEntry::Invalid(-ENOSYS)
	}

	/// `mkdir` creates a directory entry
	fn mkdir(&self, _name: *const u8, _mode: u32) -> i32 {
		-EINVAL
	}

	/// `accept` a connection on a socket
	#[cfg(all(any(feature = "tcp", feature = "udp"), not(feature = "newlib")))]
	fn accept(&self, _addr: *mut sockaddr, _addrlen: *mut socklen_t) -> i32 {
		-EINVAL
	}

	/// initiate a connection on a socket
	#[cfg(all(any(feature = "tcp", feature = "udp"), not(feature = "newlib")))]
	fn connect(&self, _name: *const sockaddr, _namelen: socklen_t) -> i32 {
		-EINVAL
	}

	/// `bind` a name to a socket
	#[cfg(all(any(feature = "tcp", feature = "udp"), not(feature = "newlib")))]
	fn bind(&self, _name: *const sockaddr, _namelen: socklen_t) -> i32 {
		-EINVAL
	}

	/// `listen` for connections on a socket
	#[cfg(all(any(feature = "tcp", feature = "udp"), not(feature = "newlib")))]
	fn listen(&self, _backlog: i32) -> i32 {
		-EINVAL
	}

	/// `setsockopt` sets options on sockets
	#[cfg(all(any(feature = "tcp", feature = "udp"), not(feature = "newlib")))]
	fn setsockopt(
		&self,
		_level: i32,
		_optname: i32,
		_optval: *const c_void,
		_optlen: socklen_t,
	) -> i32 {
		-EINVAL
	}

	/// `getsockopt` gets options on sockets
	#[cfg(all(any(feature = "tcp", feature = "udp"), not(feature = "newlib")))]
	fn getsockopt(
		&self,
		_level: i32,
		_option_name: i32,
		_optval: *mut c_void,
		_optlen: *mut socklen_t,
	) -> i32 {
		-EINVAL
	}

	/// `getsockname` gets socket name
	#[cfg(all(any(feature = "tcp", feature = "udp"), not(feature = "newlib")))]
	fn getsockname(&self, _name: *mut sockaddr, _namelen: *mut socklen_t) -> i32 {
		-EINVAL
	}

	/// `getpeername` get address of connected peer
	#[cfg(all(any(feature = "tcp", feature = "udp"), not(feature = "newlib")))]
	fn getpeername(&self, _name: *mut sockaddr, _namelen: *mut socklen_t) -> i32 {
		-EINVAL
	}

	/// receive a message from a socket
	///
	/// If `address` is not a null pointer, the source address of the message is filled in.  The
	/// `address_len` argument is a value-result argument, initialized to the size
	/// of the buffer associated with address, and modified on return to
	/// indicate the actual size of the address stored there.
	#[cfg(all(any(feature = "tcp", feature = "udp"), not(feature = "newlib")))]
	fn recvfrom(
		&self,
		_buffer: *mut u8,
		_len: usize,
		_address: *mut sockaddr,
		_address_len: *mut socklen_t,
	) -> isize {
		(-ENOSYS).try_into().unwrap()
	}

	/// send a message from a socket
	///
	/// The sendto() function shall send a message.
	/// If the socket is a connectionless-mode socket, the message shall
	/// If a peer address has been prespecified, either the message shall
	/// be sent to the address specified by dest_addr (overriding the pre-specified peer
	/// address).
	#[cfg(all(any(feature = "tcp", feature = "udp"), not(feature = "newlib")))]
	fn sendto(
		&self,
		_buffer: *const u8,
		_len: usize,
		_addr: *const sockaddr,
		_addr_len: socklen_t,
	) -> isize {
		(-ENOSYS).try_into().unwrap()
	}

	/// shut down part of a full-duplex connection
	#[cfg(all(any(feature = "tcp", feature = "udp"), not(feature = "newlib")))]
	fn shutdown(&self, _how: i32) -> i32 {
		-EINVAL
	}

	/// The `ioctl` function manipulates the underlying device parameters of special
	/// files.
	fn ioctl(&self, _cmd: i32, _argp: *mut c_void) -> i32 {
		-EINVAL
	}
}

pub(crate) fn open(name: *const u8, flags: i32, mode: i32) -> Result<FileDescriptor, i32> {
	if env::is_uhyve() {
		let mut sysopen = SysOpen::new(VirtAddr(name as u64), flags, mode);
		uhyve_send(UHYVE_PORT_OPEN, &mut sysopen);

		if sysopen.ret > 0 {
			let fd = FD_COUNTER.fetch_add(1, Ordering::SeqCst);
			let file = UhyveFile::new(sysopen.ret);

			if OBJECT_MAP.write().try_insert(fd, Arc::new(file)).is_err() {
				Err(-EINVAL)
			} else {
				Ok(fd as FileDescriptor)
			}
		} else {
			Err(sysopen.ret)
		}
	} else {
		{
			// mode is 0x777 (0b0111_0111_0111), when flags | O_CREAT, else 0
			// flags is bitmask of O_DEC_* defined above.
			// (taken from rust stdlib/sys hermit target )

			let name = unsafe { CStr::from_ptr(name as _) }.to_str().unwrap();
			debug!("Open {}, {}, {}", name, flags, mode);

			let mut fs = fs::FILESYSTEM.lock();
			if let Ok(filesystem_fd) = fs.open(name, open_flags_to_perm(flags, mode as u32)) {
				let fd = FD_COUNTER.fetch_add(1, Ordering::SeqCst);
				let file = GenericFile::new(filesystem_fd);
				if OBJECT_MAP.write().try_insert(fd, Arc::new(file)).is_err() {
					Err(-EINVAL)
				} else {
					Ok(fd as FileDescriptor)
				}
			} else {
				Err(-EINVAL)
			}
		}
	}
}

#[allow(unused_variables)]
pub(crate) fn opendir(name: *const u8) -> Result<FileDescriptor, i32> {
	if env::is_uhyve() {
		Err(-EINVAL)
	} else {
		#[cfg(target_arch = "x86_64")]
		{
			let name = unsafe { CStr::from_ptr(name as _) }.to_str().unwrap();
			debug!("Open directory {}", name);

			let mut fs = fs::FILESYSTEM.lock();
			if let Ok(filesystem_fd) = fs.opendir(name) {
				let fd = FD_COUNTER.fetch_add(1, Ordering::SeqCst);
				// Would a GenericDir make sense?
				let file = GenericFile::new(filesystem_fd);
				if OBJECT_MAP.write().try_insert(fd, Arc::new(file)).is_err() {
					Err(-EINVAL)
				} else {
					Ok(fd as FileDescriptor)
				}
			} else {
				Err(-EINVAL)
			}
		}
		#[cfg(not(target_arch = "x86_64"))]
		{
			Err(-ENOSYS)
		}
	}
}

pub(crate) fn get_object(fd: FileDescriptor) -> Result<Arc<dyn ObjectInterface>, i32> {
	Ok((*(OBJECT_MAP.read().get(&fd).ok_or(-EINVAL)?)).clone())
}

#[cfg(all(any(feature = "tcp", feature = "udp"), not(feature = "newlib")))]
pub(crate) fn insert_object(
	fd: FileDescriptor,
	obj: Arc<dyn ObjectInterface>,
) -> Option<Arc<dyn ObjectInterface>> {
	OBJECT_MAP.write().insert(fd, obj)
}

// The dup system call allocates a new file descriptor that refers
// to the same open file description as the descriptor oldfd. The new
// file descriptor number is guaranteed to be the lowest-numbered
// file descriptor that was unused in the calling process.
pub(crate) fn dup_object(fd: FileDescriptor) -> Result<FileDescriptor, i32> {
	let mut guard = OBJECT_MAP.write();
	let obj = (*(guard.get(&fd).ok_or(-EINVAL)?)).clone();

	let new_fd = || -> i32 {
		for i in 3..FD_COUNTER.load(Ordering::SeqCst) {
			if !guard.contains_key(&i) {
				return i;
			}
		}
		FD_COUNTER.fetch_add(1, Ordering::SeqCst)
	};

	let fd = new_fd();
	if guard.try_insert(fd, obj).is_err() {
		Err(-EMFILE)
	} else {
		Ok(fd as FileDescriptor)
	}
}

pub(crate) fn remove_object(fd: FileDescriptor) -> Result<Arc<dyn ObjectInterface>, i32> {
	if fd <= 2 {
		Err(-EINVAL)
	} else {
		let obj = OBJECT_MAP.write().remove(&fd).ok_or(-EINVAL)?;
		Ok(obj)
	}
}

pub(crate) fn init() {
	let mut guard = OBJECT_MAP.write();
	if env::is_uhyve() {
		guard
			.try_insert(STDIN_FILENO, Arc::new(UhyveStdin::new()))
			.unwrap();
		guard
			.try_insert(STDOUT_FILENO, Arc::new(UhyveStdout::new()))
			.unwrap();
		guard
			.try_insert(STDERR_FILENO, Arc::new(UhyveStderr::new()))
			.unwrap();
	} else {
		guard
			.try_insert(STDIN_FILENO, Arc::new(GenericStdin::new()))
			.unwrap();
		guard
			.try_insert(STDOUT_FILENO, Arc::new(GenericStdout::new()))
			.unwrap();
		guard
			.try_insert(STDERR_FILENO, Arc::new(GenericStderr::new()))
			.unwrap();
	}
}
