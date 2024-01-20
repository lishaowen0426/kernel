use core::cell::{RefCell, RefMut};
use core::mem::MaybeUninit;
use core::result::Result;

use hermit_sync::{Lazy, SpinMutex};
use wasmi::{Engine, Module, Store};

use super::*;

pub static WASM: Lazy<SpinMutex<WasmManager>> = Lazy::new(|| SpinMutex::new(WasmManager::new()));

const MODULE_COUNT: usize = 32;
pub struct WasmManager {
	engine: Engine,
	module: [MaybeUninit<RefCell<WasmModule>>; MODULE_COUNT],
	next: WasmHandle,
}

pub struct WasmStoreData {}

pub struct WasmModule {
	module: Module,
	store: Store<WasmStoreData>,
	handle: WasmHandle,
}
pub type WasmHandle = usize;

impl WasmStoreData {
	fn new() -> Self {
		Self {}
	}
}

impl WasmModule {
	fn new(engine: &Engine, handle: WasmHandle, wasm: &[u8]) -> Result<Self, WasmError> {
		let module = Module::new(engine, wasm).map_err(|e| WasmError::Module(e))?;
		let store = Store::new(engine, WasmStoreData::new());
		Ok(Self {
			module,
			store,
			handle,
		})
	}

	pub fn instantiate(&mut self) -> Result<(), WasmError> {
		info!("Imports: ");
		for i in self.module.imports() {
			info!("{:?}", i);
		}
		info!("Export: ");
		for i in self.module.exports() {
			info!("{:?}", i);
		}

		Ok(())
	}
}

// currently not support remove
impl WasmManager {
	fn new() -> Self {
		Self {
			engine: Engine::default(),
			module: MaybeUninit::uninit_array(),
			next: 0,
		}
	}

	pub fn load(&mut self, wasm: &[u8]) -> Result<WasmHandle, WasmError> {
		if self.next >= MODULE_COUNT {
			Err(WasmError::Full)
		} else {
			let handle = self.next;
			self.next += 1;
			self.module[handle].write(RefCell::new(WasmModule::new(&self.engine, handle, wasm)?));
			Ok(handle)
		}
	}

	pub unsafe fn get_mut(&mut self, handle: WasmHandle) -> RefMut<'_, WasmModule> {
		self.module[handle].assume_init_ref().borrow_mut()
	}
}
