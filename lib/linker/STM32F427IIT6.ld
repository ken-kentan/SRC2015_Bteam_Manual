/* Linker script Upperside for STM32F427IIT6! 				*/
/* Nemui Trinomius (http://nemuisan.blog.bai.ne.jp) 		*/
/* Based on STMicro's RIDE Project Sample thanks!			*/

OUTPUT_FORMAT ("elf32-littlearm")

/* Memory Spaces Definitions */
MEMORY
{
	RAM1 (xrw)	: ORIGIN = 0x20000000, LENGTH = 128k	/* Main Embedded SRAM 					*/
	RAM2 (xrw)	: ORIGIN = 0x20020000, LENGTH = 64k		/* Main Embedded SRAM (usually DMA use) */
	RAM3 (xrw)	: ORIGIN = 0x10000000, LENGTH = 64k		/* Core-Coupled SRAM 					*/
	RAM4 (xrw)	: ORIGIN = 0x40024000, LENGTH = 4K		/* Backuped SRAM 						*/
	RAM5 (xrw)	: ORIGIN = 0x68000000, LENGTH = 2M		/* External SRAM (FMC)	 				*/
	ROM (rx)	: ORIGIN = 0x08000000, LENGTH = 2M		/* Main Embedded FlashROM				*/
}

/* higher address of the stack bottom */
_estack = ORIGIN(RAM3)+LENGTH(RAM3);

/* higher address of the heap end */
_heap_end = ORIGIN(RAM1)+LENGTH(RAM1)-4;	/* due to 4byte alignments */


/* include the section management sub-script */
INCLUDE "STM32F42_43xxx_FLASH.ld"
