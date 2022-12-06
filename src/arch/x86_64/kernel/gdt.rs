use alloc::boxed::Box;
use core::sync::atomic::Ordering;

use x86_64::instructions::tables;
use x86_64::registers::segmentation::{Segment, CS, DS, ES, SS};
use x86_64::structures::gdt::{Descriptor, GlobalDescriptorTable};
use x86_64::structures::tss::TaskStateSegment;
use x86_64::VirtAddr;

use super::interrupts::{IST_ENTRIES, IST_SIZE};
use super::scheduler::TaskStacks;
use super::CURRENT_STACK_ADDRESS;
use crate::arch::x86_64::kernel::percore::*;
use crate::config::*;

pub fn add_current_core() {
	let gdt = Box::leak(Box::new(GlobalDescriptorTable::new()));
	let kernel_code_selector = gdt.add_entry(Descriptor::kernel_code_segment());
	let kernel_data_selector = gdt.add_entry(Descriptor::kernel_data_segment());

	// Dynamically allocate memory for a Task-State Segment (TSS) for this core.
	let mut boxed_tss = Box::new(TaskStateSegment::new());

	// Every task later gets its own stack, so this boot stack is only used by the Idle task on each core.
	// When switching to another task on this core, this entry is replaced.
	let rsp = CURRENT_STACK_ADDRESS.load(Ordering::Relaxed) + KERNEL_STACK_SIZE as u64
		- TaskStacks::MARKER_SIZE as u64;
	boxed_tss.privilege_stack_table[0] = VirtAddr::new(rsp);
	set_kernel_stack(rsp);

	// Allocate all ISTs for this core.
	// Every task later gets its own IST1, so the IST1 allocated here is only used by the Idle task.
	for i in 0..IST_ENTRIES {
		let ist = crate::mm::allocate(IST_SIZE, true);
		let ist_start = ist.as_u64() + IST_SIZE as u64 - TaskStacks::MARKER_SIZE as u64;
		boxed_tss.interrupt_stack_table[i] = VirtAddr::new(ist_start);
	}

	let tss = Box::into_raw(boxed_tss);
	unsafe {
		PERCORE.tss.set(tss);
	}
	let tss = unsafe { &*(tss as *mut x86_64::structures::tss::TaskStateSegment) };
	let tss_selector = gdt.add_entry(Descriptor::tss_segment(tss));

	unsafe {
		// Load the GDT for the current core.
		gdt.load();

		// Reload the segment descriptors
		CS::set_reg(kernel_code_selector);
		DS::set_reg(kernel_data_selector);
		ES::set_reg(kernel_data_selector);
		SS::set_reg(kernel_data_selector);
		tables::load_tss(tss_selector);
	}
}

pub extern "C" fn set_current_kernel_stack() {
	core_scheduler().set_current_kernel_stack();
}
