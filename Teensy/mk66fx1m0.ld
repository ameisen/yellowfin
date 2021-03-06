/* Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2017 PJRC.COM, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

 /* --fix-stm32l4xx-629360 */

MEMORY
{
	FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 1024K
	SRAM  (rwx) : ORIGIN = 0x1FFF0000, LENGTH = 256K
	FlexRAM (rw) : ORIGIN = 0x14000000, LENGTH = 4K
}

/* REGION_ALIAS */

SECTIONS
{
	.text : {
		. = ORIGIN(FLASH);
		KEEP(*(.vectors))
		*(.startup*)
		/* TODO: does linker detect startup overflow onto flashconfig? */
		. = 0x400;
		KEEP(*(.flashconfig*))
		*(.glue_7t*)
		*(.glue_7*)
		*(.text*)
		*(.rodata*)
		. = ALIGN(4);
		KEEP(*(.init))
		. = ALIGN(4);
		__preinit_array_start = .;
		KEEP (*(.preinit_array))
		__preinit_array_end = .;
		__init_array_start = .;
		KEEP (*(SORT(.init_array.*)))
		KEEP (*(.init_array))
		__init_array_end = .;
	} > FLASH = 0xFF

	.ARM.exidx : {
		__exidx_start = .;
		*(.ARM.exidx* .gnu.linkonce.armexidx.*)
		__exidx_end = .;
	} > FLASH
	_etext = .;

	.usbdescriptortable (NOLOAD) : {
		/* . = ORIGIN(SRAM); */
		. = ALIGN(512);
		*(.usbdescriptortable*)
	} > SRAM

	.dmabuffers (NOLOAD) : {
		. = ALIGN(4);
		*(.dmabuffers*)
	} > SRAM

	.usbbuffers (NOLOAD) : {
		. = ALIGN(4);
		*(.usbbuffers*)
	} > SRAM

	/* Is there a way to sanely handle the 64KiB / 192KiB break? */
	.data : AT (_etext) {
		. = ALIGN(4);
		_sdata = .;
		*(.flexnvm*)
		*(.fastrun*)
		*(.data*)
		. = ALIGN(4);
		_edata = .; 
	} > SRAM
	_datasize = SIZEOF(.data);

	.noinit (NOLOAD) : {
		*(.noinit*)
	} > SRAM

	.bss : {
		. = ALIGN(4);
		_sbss = .;
		__bss_start__ = .;
		*(.bss*)
		*(COMMON)
		. = ALIGN(4);
		_ebss = .;
		_bsssize = _ebss - _sbss;
		__bss_end = .;
		__bss_end__ = .;
	} > SRAM
	_bsssize = SIZEOF(.bss);

	/* TODO add ASSERT(c, m) to validate enough space is left for stack */
	_estack = ORIGIN(SRAM) + LENGTH(SRAM);

	.flex (NOLOAD) : {
		. = ORIGIN(FlexRAM);
		*(.flexram*)
	} > FlexRAM

	/DISCARD/ : {
		*(.ARM.attributes*)
		*(.vfp11_veneer*)
		*(.v4_bx*)
		*(*.iplt*)
		*(*.fini*)
		/**(*.eh_*)*/
	}
}




