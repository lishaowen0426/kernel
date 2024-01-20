use alloc::vec::Vec;

use generic_array::typenum::{U512, U8};
use littlefs2::driver::Storage;
use virtio_drivers::device::blk::VirtIOBlk;
use virtio_drivers::transport::pci::PciTransport;

use crate::drivers::pci::VirtioHal;
use crate::fs::fatfs::{self, IoBase, IoError};

const BLOCK_SIZE: usize = 512;

pub type VirtioBlk = VirtIOBlk<VirtioHal, PciTransport>;
pub struct VirtioBlkDriver {
	blk: VirtioBlk,
	seek: usize,
	eof: usize,
}

impl VirtioBlkDriver {
	pub fn new(blk: VirtioBlk) -> Self {
		Self {
			blk,
			seek: 0,
			eof: 0, // 1 past the last byte, i.e., length of the data
		}
	}
}

#[derive(Debug, Clone, Copy)]
pub enum VirtioBlkIoError {
	UnexpectedEof,
	WriteZero,
	Flush,
}

impl IoError for VirtioBlkIoError {
	fn is_interrupted(&self) -> bool {
		false
	}

	fn new_unexpected_eof_error() -> Self {
		Self::UnexpectedEof
	}

	fn new_write_zero_error() -> Self {
		Self::WriteZero
	}
}

impl IoBase for VirtioBlkDriver {
	type Error = VirtioBlkIoError;
}

fn align_up(value: usize, alignment: usize) -> usize {
	((value - 1) | (alignment - 1)) + 1
}
impl fatfs::Read for VirtioBlkDriver {
	fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
		if buf.len() == 0 || self.seek == self.eof {
			return Ok(0);
		}

		let start_blk = self.seek / BLOCK_SIZE;
		assert!(start_blk < self.blk.capacity() as usize); //

		let bytes_to_copy = if self.seek + buf.len() < self.eof {
			buf.len()
		} else {
			self.eof - self.seek
		};
		assert!(bytes_to_copy > 0);

		let blks_to_copy = align_up(bytes_to_copy, BLOCK_SIZE) / BLOCK_SIZE;

		let mut storage = [0xffu8; BLOCK_SIZE];
		let mut data: Vec<u8> = Vec::new();

		for i in 0..blks_to_copy {
			self.blk.read_blocks(start_blk + i, &mut storage).unwrap();
			data.extend_from_slice(&storage);
		}

		assert!(data.len() == blks_to_copy * BLOCK_SIZE);

		for (from, to) in data[self.seek % BLOCK_SIZE..].iter().zip(buf.iter_mut()) {
			*to = *from;
		}

		//adjust seek
		self.seek += bytes_to_copy;
		assert!(self.seek <= self.eof);

		Ok(blks_to_copy)
	}
}

impl fatfs::Write for VirtioBlkDriver {
	fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
		let capacity = self.blk.capacity() as usize * BLOCK_SIZE;

		if buf.len() == 0 || self.seek == capacity {
			return Ok(0);
		}

		let start_blk = self.seek / BLOCK_SIZE;
		let bytes_to_write = if self.seek + buf.len() < capacity {
			buf.len()
		} else {
			capacity - self.seek
		};
		let blks_to_write = align_up(bytes_to_write, BLOCK_SIZE) / BLOCK_SIZE;
		assert!(blks_to_write > 0);

		let mut data: Vec<u8> = Vec::new();

		let mut head = [0xffu8; BLOCK_SIZE];
		self.blk.read_blocks(start_blk, &mut head).unwrap();
		let mut tail = [0xffu8; BLOCK_SIZE];
		self.blk
			.read_blocks(start_blk + blks_to_write - 1, &mut tail)
			.unwrap();

		data.extend_from_slice(&head[0..(self.seek % BLOCK_SIZE)]);
		data.extend_from_slice(&buf[0..bytes_to_write]);
		data.extend_from_slice(&tail[(self.seek + bytes_to_write) % BLOCK_SIZE..]);

		assert!(data.len() == blks_to_write * BLOCK_SIZE);

		for i in 0..blks_to_write {
			self.blk
				.write_blocks(start_blk + i, &data[i * BLOCK_SIZE..(i + 1) * BLOCK_SIZE])
				.unwrap()
		}

		self.seek += bytes_to_write;

		assert!(self.seek <= capacity);
		if self.eof < self.seek {
			self.eof = self.seek;
		}

		Ok(bytes_to_write)
	}

	fn flush(&mut self) -> Result<(), Self::Error> {
		self.blk.flush().map_err(|_| VirtioBlkIoError::Flush)
	}
}

impl fatfs::Seek for VirtioBlkDriver {
	fn seek(&mut self, pos: fatfs::SeekFrom) -> Result<u64, Self::Error> {
		self.seek = match pos {
			fatfs::SeekFrom::Start(i) => i as usize,
			fatfs::SeekFrom::Current(i) => (self.seek as i64 + i).try_into().unwrap(),
			fatfs::SeekFrom::End(i) => (self.eof as i64 + i).try_into().unwrap(),
		};

		assert!(self.seek < (self.blk.capacity() as usize * BLOCK_SIZE));
		//adjust eof
		if self.seek > self.eof {
			self.eof = self.seek
		}
		Ok(self.seek as u64)
	}
}

static ERASE_BUFFER: [u8; BLOCK_SIZE] = [0xffu8; BLOCK_SIZE];
impl Storage for VirtioBlkDriver {
	const READ_SIZE: usize = BLOCK_SIZE;

	const WRITE_SIZE: usize = BLOCK_SIZE;

	const BLOCK_SIZE: usize = BLOCK_SIZE;

	const BLOCK_COUNT: usize = 262144;

	type CACHE_SIZE = U512;

	type LOOKAHEAD_SIZE = U8;

	fn read(&mut self, off: usize, buf: &mut [u8]) -> littlefs2::io::Result<usize> {
		assert!(off % Self::READ_SIZE == 0);
		assert!(buf.len() % Self::READ_SIZE == 0);

		let start_blk = off / Self::READ_SIZE;
		for (blk, c) in buf.chunks_mut(Self::READ_SIZE).enumerate() {
			self.blk.read_blocks(start_blk + blk, c).unwrap();
		}
		Ok(buf.len())
	}

	fn write(&mut self, off: usize, data: &[u8]) -> littlefs2::io::Result<usize> {
		assert!(off % Self::WRITE_SIZE == 0);
		assert!(data.len() % Self::WRITE_SIZE == 0);

		let start_blk = off / Self::READ_SIZE;
		for (blk, c) in data.chunks(Self::WRITE_SIZE).enumerate() {
			self.blk.write_blocks(start_blk + blk, c).unwrap();
		}
		Ok(data.len())
	}

	fn erase(&mut self, off: usize, len: usize) -> littlefs2::io::Result<usize> {
		assert!(off % Self::BLOCK_SIZE == 0);
		assert!(len % Self::BLOCK_SIZE == 0);

		let start_blk = off / Self::BLOCK_SIZE;
		for i in 0..(len / Self::BLOCK_SIZE) {
			self.blk
				.write_blocks(start_blk + i, ERASE_BUFFER.as_slice())
				.unwrap();
		}
		Ok(len)
	}
}
