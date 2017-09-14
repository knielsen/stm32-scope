MEMORY
{
	FLASH (rx) : ORIGIN = 0x8000000, LENGTH = 1024K
	RAM_MAIN (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
	RAM_CPU (rwx) : ORIGIN = 0x10000000, LENGTH = 64K
}
_estack = 0x10010000;


SECTIONS
{
	.text :
	{
		KEEP(*(.isr_vector))
		*(.text*)
		*(.rodata*)

		/* Some libc code for static constructors (or something) goes here. */
		*(.init*)
		*(.fini*)

		. = ALIGN(4);
		_etext = .;
	} > FLASH

	_sidata = .;

	.data : AT(_sidata)
	{
		_sdata = .;
		*(.data*)
		. = ALIGN(4);
		_edata = .;
	} > RAM_MAIN

	.bss :
	{
		_sbss = .;
		*(.bss*)
		*(COMMON)
		. = ALIGN(4);
		_ebss = .;
	} > RAM_MAIN

	/* Explicitly place stuff in the STM32F4 core-coupled memory. This
	memory will not be initialised. And it is not available to peripherals
	(such as DMA). */
	.uninit_ccm (NOLOAD) :
	{
		*(.uninit_ccm_data)
		. = ALIGN(4);
	} > RAM_CPU

	/* Check that there is room for stack. */
	._stack_free_space :
	{
		. = . + 4K;  /* Minimum 4 kByte stack free. */
	} > RAM_CPU
}

ENTRY(Reset_Handler)
