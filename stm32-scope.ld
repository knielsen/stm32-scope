MEMORY
{
	FLASH (rx) : ORIGIN = 0x8000000, LENGTH = 1024K
	RAM_MAIN (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
	RAM_CPU (rwx) : ORIGIN = 0x10000000, LENGTH = 64K
}
_estack = 0x20020000;


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

	/* Check that there is room for stack. */
	._stack_free_space :
	{
		. = . + 4K;  /* Minimum 4 kByte stack free. */
	} > RAM_MAIN
}

ENTRY(Reset_Handler)
