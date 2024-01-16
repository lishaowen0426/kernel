use alloc::string::String;
use core::convert::Into;
use core::fmt::{Debug, Display};
const WASM_FILE: &'static [u8; 2517579] = include_bytes!(
	"/Users/lsw/Code/hermit/wasi_hello_world/target/wasm32-wasi/debug/wasi_hello_world.wasm"
);

mod host_api;
mod manager;
use manager::WASM;
use wasmi;
pub enum WasmError {
	Full,
	Module(wasmi::Error),
	General(String),
}

impl Into<i32> for WasmError {
	fn into(self) -> i32 {
		-1
	}
}

impl Debug for WasmError {
	fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
		match self {
			Self::Full => write!(f, "module full"),
			Self::Module(e) => Debug::fmt(e, f),
			Self::General(e) => write!(f, "{}", e),
		}
	}
}

impl Display for WasmError {
	fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
		core::fmt::Debug::fmt(&self, f)
	}
}

pub fn init() {
	info!("engine is running");
}

pub extern "C" fn workloop(_arg: usize) -> i32 {
	let mut locked = WASM.lock();
	let handle = match locked.load(WASM_FILE.as_slice()) {
		Ok(h) => h,
		Err(e) => return e.into(),
	};
	info!("handle: {}", handle);

	unsafe {
		let mut module = locked.get_mut(handle);
		if let Err(e) = module.instantiate() {
			return e.into();
		}
	}

	0
}
