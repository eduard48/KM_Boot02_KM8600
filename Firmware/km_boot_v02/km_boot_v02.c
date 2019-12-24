/******************************************************************************
 * km_boot_v02.c
 *
 * Created: 21.02.2015 10:39:39
 *  Author: Ed
 *****************************************************************************/ 

/*****************************************************************************
*
* AVRPROG compatible boot-loader
* Version  : 2.0
* Compiler : avr-gcc 4.1.2 / avr-libc 1.4.6
* size     : depends on features and startup ( ����������� ������ < 512 ����)
* by       : ��������� ���������� �� M. Thomas (2008)
*
****************************************************************************
*	��������� ���������� v02 ��� ������������ �� (ATMEGA324P)
* ���� � ��������� ������� ������� ������������ ������� Makefile!!!
*  ������� makefile � ���������, ��� ������� ��������� ����������.  
*  ����� ������� ������ Boot Size (BOOTSIZE=xxxx) � ��� CPU
*  ��������� ������������ ��������� � ���� �����.
*
*  ��������� ����� ����������� ������ 512 ���� ��� (1024, 0x400 ������) 
*  �.�. �� ������ bootloader-section ���������� �������� 1024 �����. 
*
* �������� ��� ������������ SEM ��01-8600.� � ��01-2442.� - ��������!!!
*
* ------------- �������� ������ --------------------------------------------------
* ���������� BOOTCZ = 11 (1k boot) � BOOTRST = 1 (��� ������, ������ ����� � boot)
* ����� ���� �������� ����� ������ ���������� � 2 ���� ������ ��� � Makefile!!!
****************************************************************************/


/*  �������� ������� */
#ifndef F_CPU
//#define F_CPU 7372800
#define F_CPU 14745600
#endif

/* UART �������� UART ���������� 19200 */
//#define BAUDRATE 9600
#define BAUDRATE 19200

/***************************************************
* ��� ����������:
*   ��� AVRProg �������� BOOT 
*   ��� ���������� �������� ��� bootloader.
*   avrdude ����� ���������� ������ part-code ��� ISP 
****************************************************/
#define DEVTYPE     DEVTYPE_BOOT
// #define DEVTYPE     DEVTYPE_ISP // ��� �� ���������

/***************************************************
 * ����� ����� ��� �������� ����� � ���������
 * ����� ����� � ��������� ���� ����� ��� ������� ���� ������� ��� ������� �� +5�����
 * ��� 1 � 2 ��� ������� ISP!
 ***************************************************/
 
#define BLPORT		PORTB
#define BLDDR		DDRB
#define BLPIN		PINB
#define BLPNUM		PINB6

/***************************************************
 * ����� ����� ��� ���������� ������ ����������
 * ��������� ����� - �� � ���������� (��������� ������)
****************************************************/

//#define ENABLE_BOOT_LED
#define BIPORT		PORTB
#define BIDDR		DDRB
#define BIPIN		PINB
#define BIPNUM		PINB3

/****************************************************
* ����� ����� ������ ���������� ���������
* �� RS485 (��������� ������)
*****************************************************/

//#define RS485
#define RSPORT	PORTD
#define RSDDR	DDRD
#define RSPNUM	PIND2

/****************************************************
 * ��������� ������� ������ �� ����� ����������
*****************************************************/
#define DISABLE_WDT_AT_STARTUP

/****************************************************
 * Watchdog-reset is issued at exit 
 * define the timeout-value here (see avr-libc manual)
*****************************************************/
#define EXIT_WDT_TIME   WDTO_250MS

/****************************************************
 * ����� ������ ����������
 * SIMPLE-Mode - ��������� �������� ����� �� ������� ������ +5�
 *   ������� � �������� ��������� �������������� ����� ������ 
 *   (������� ����� ���� ���������) ���� �� ������� �� �������������
 *   ��� ���� ������ ����� �� ������� ��������������� ��� ����-� ��� �������,
 *   �� ��� ������ �� ���������� ��� ������������ �� ���������
 * BOOTICE-Mode - ��� �������  JTAGICE ����� upgrade.ebn � ����16.
 *   ��� ���������� �� � JTAG ��������. ���������� ����� �������� ���� �����������
 *   ����� �� �������� ��� �����. � ������� ������ ���� ����� ��������� ��� 7372800
 *   � F_CPU ��� ������������� � ������ ��������� JTAG ICE
 * WAIT-mode Bootloader ������� ������� �� ����, ���� �� �� ���� � ������� ���������� �������
 *   (������� �������������) �� ���������� ������� � �������� ���������.
 *****************************************************/
//#define START_SIMPLE
#define START_WAIT
//#define START_BOOTICE

/* ������� ��� ����� � ��������� � START_WAIT */
#define START_WAIT_UARTCHAR 'S'

/* �������� ��� START_WAIT mode ( t = WAIT_TIME * 10ms ) */
#define WAIT_VALUE 400 /* ������: 400*10ms = 3000ms = 3sec */

/******************************************************************
* ������ ������ ��������� bootloader ��� ����� ��������� bootloader
* ������ �� ������ flash (�������� ������ �������� (0xFFFF))
*******************************************************************/
//#define READ_PROTECT_BOOTLOADER

#define VERSION_HIGH '0'
#define VERSION_LOW  '8'

#define GET_LOCK_BITS           0x0001
#define GET_LOW_FUSE_BITS       0x0000
#define GET_HIGH_FUSE_BITS      0x0003
#define GET_EXTENDED_FUSE_BITS  0x0002

/* ������ �������� ������� ��� USART*/
#define UART_CALC_BAUDRATE(baudRate) ((uint32_t)((F_CPU) + ((uint32_t)baudRate * 8UL)) / ((uint32_t)(baudRate) * 16UL) - 1)

/* ���������� ����� */

#include <stdint.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/boot.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "km_boot_v02.h"

uint8_t gBuffer[SPM_PAGESIZE];

#if defined(BOOTLOADERHASNOVECTORS)
	#warning "This Bootloader does not link interrupt vectors - see makefile"
	/* make the linker happy - it wants to see __vector_default */
	// void __vector_default(void) { ; }
	void __vector_default(void) { ; }
#endif

static void sendchar(uint8_t data)
{
	RSPORT |= (1<<RSPNUM);							// 1 �� ������ TXE
	UART_DATA = data;								// �������� ���� � �����
	while (!(UART_STATUS & (1<<UART_TXREADY))); 	// ����� ����� ��������
	UART_STATUS |= (1<<UART_TXREADY);				// �������� ���� �.�. ��� ����������
	_delay_us(500);									// �������� ��� �������� ���������� ����� (�� ������� �� ��������!)
	RSPORT &= ~(1<<RSPNUM);						// 0 �� ������ TXE
}

static uint8_t recvchar(void)
{
	while (!(UART_STATUS & (1<<UART_RXREADY))); 	// ����� ������
	return UART_DATA;								// ��������� � �������� ��������
}

static inline void eraseFlash(void)
{
	// �������� ������ �������� ��������� (bootloader ������� (�� ���������))
	uint32_t addr = 0;
	while (APP_END > addr) 
		{
		boot_page_erase(addr);		// ��������� �������� ��������
		boot_spm_busy_wait();		// ����� ����� �������� ��������.
		addr += SPM_PAGESIZE;		// ������� � ��������� ��������
		}
	boot_rww_enable();
}

static inline void recvBuffer(pagebuf_t size)
{
	pagebuf_t cnt;
	uint8_t *tmp = gBuffer;

	for (cnt = 0; cnt < sizeof(gBuffer); cnt++) 
		{
		*tmp++ = (cnt < size) ? recvchar() : 0xFF;
		}
}

static inline uint16_t writeFlashPage(uint16_t waddr, pagebuf_t size)
{
	uint32_t pagestart = (uint32_t)waddr<<1;
	uint32_t baddr = pagestart;
	uint16_t data;
	uint8_t *tmp = gBuffer;

	do 
		{
		data = *tmp++;
		data |= *tmp++ << 8;
		boot_page_fill(baddr, data);	// ����� asm ������������.

		baddr += 2;			// ������� ��������� ����� � ������
		size -= 2;			// ��������� ��������� ���-�� ������ ��� ������
		} 
	while (size);				// ��������� ���� ��� �� ����� ��������

	boot_page_write(pagestart);
	boot_spm_busy_wait();
	boot_rww_enable();		// ������� � RWW ������

	return baddr>>1;
}

static inline uint16_t writeEEpromPage(uint16_t address, pagebuf_t size)
{
	uint8_t *tmp = gBuffer;

	do 
		{
		eeprom_write_byte( (uint8_t*)address, *tmp++ );
		address++;			// ������� ��������� ����
		size--;				// ��������� ��������� ���-�� ������ ��� ������
		}
	while (size);				// ��������� ���� ��� �� ����� ��������

	// eeprom_busy_wait();

	return address;
}

static inline uint16_t readFlashPage(uint16_t waddr, pagebuf_t size)
{
	uint32_t baddr = (uint32_t)waddr<<1;
	uint16_t data;

	do 
	{

#ifndef READ_PROTECT_BOOTLOADER
#warning "Bootloader not read-protected"

	#if defined(RAMPZ)
		data = pgm_read_word_far(baddr);
	#else
		data = pgm_read_word_near(baddr);
	#endif

#else
		// ���� ��������� ������ bootloader
		if ( baddr < APP_END ) 
		{
		#if defined(RAMPZ)
			data = pgm_read_word_far(baddr);
		#else
			data = pgm_read_word_near(baddr);
		#endif
		}
		else 
		{
		data = 0xFFFF; // ��������� ��������
		}
#endif
		sendchar(data);				// �������� ������� ���� LSB 
		sendchar((data >> 8));		// �������� ������� ���� MSB
		baddr += 2;					// ������� ��������� ����� � ������ (2 �����)
		size -= 2;					// ������� ��� ����� �� ������ ����� �������� ����
	} 
	while (size);					// ��������� ���� ���� �� ����� ��������
	return baddr>>1;
}

static inline uint16_t readEEpromPage(uint16_t address, pagebuf_t size)
{
	do 
	{
	sendchar( eeprom_read_byte( (uint8_t*)address ) );
	address++;
	size--;						// ��������� ���-�� ����������� ����
	} 
	while (size);				// ��������� ���� ���� �� ����� ��������

	return address;
}


static void send_boot(void)
{
	sendchar('A');
	sendchar('V');
	sendchar('R');
	sendchar('B');
	sendchar('O');
	sendchar('O');
	sendchar('T');
}

static void (*jump_to_app)(void) = 0x0000;

int main(void)
{
	uint16_t address = 0;
	uint8_t device = 0, val;


	BIPORT |= (1<<BIPNUM);	// ��������� �������� - LED ON (1)
	BIDDR  |= (1<<BIPNUM);

#ifdef DISABLE_WDT_AT_STARTUP // ���� ���� ��������� ������� ������ �� ����� ����������
	#ifdef WDT_OFF_SPECIAL
		#warning "using target specific watchdog_off"
		bootloader_wdt_off();
	#else
		cli();
		wdt_reset();
		wdt_disable(); // ��������� ������
	#endif
#endif
	
		// ��������� ������ ��� �������
#ifdef START_SIMPLE
	BLDDR  &= ~(1<<BLPNUM);	// ���������� ����� ������ ��� ���� (���� ��������� ����������� �� ��������)
	BLPORT |= (1<<BLPNUM);		// ��������� ���� � +
#endif
	// ��������� ����� ���������� TXE ��� RS485
	RSPORT &= ~(1<<RSPNUM); 	// �� ������ ���������� 0 (TXE - �� �������)
	RSDDR |= (1<<RSPNUM);		// ����� ��� �����	
	// ���������� �������� ����� UART
	UART_BAUD_HIGH = (UART_CALC_BAUDRATE(BAUDRATE)>>8) & 0xFF;
	UART_BAUD_LOW = (UART_CALC_BAUDRATE(BAUDRATE) & 0xFF);

	UART_CTRL = UART_CTRL_DATA;		// �������� RX, TX
	UART_CTRL2 = UART_CTRL2_DATA;	// ��������� ��������� ������ 8,n,1
	

#if defined(START_SIMPLE) // ���� ������ �� �������� !!!!!!!!!!!!!!!!!!!!!!! (��� ������)

	if ((BLPIN & (1<<BLPNUM))) {
		// ������� � ��������� ���������� ���� �� ����� ������ �� 0
		BLPORT &= ~(1<<BLPNUM);	// ���������� �� ��������� (�� �����������)

		BIPORT &= ~(1<<BIPNUM);	// ��������� ��������� � ���������� ����� �� ���������
		BIDDR  &= ~(1<<BIPNUM);

		RSPORT &= ~(1<<RSPNUM);	// ����� TXE ��� ����, �� ���������
		RSDDR  &= ~(1<<RSPNUM);	

		jump_to_app();				// ������� � ������ ����������
	}

#elif defined(START_WAIT)	// ���� ������ �� �������!!!!!!!!!!!!!!!!!!!!!! (��� ������)

	uint16_t cnt = 0;

	while (1) {
		if (UART_STATUS & (1<<UART_RXREADY))
			if (UART_DATA == START_WAIT_UARTCHAR)
				break;

		if (cnt++ >= WAIT_VALUE) {		// ���� ����� �������� �����!
			BLPORT &= ~(1<<BLPNUM);	// ���������� ��� �� ���������

			BIPORT &= ~(1<<BIPNUM);	// ��������� ��������� � ���������� ����� �� ���������
			BIDDR  &= ~(1<<BIPNUM);

			jump_to_app();			// ������� � ������ ����������
		}

		_delay_ms(10);					// �������� ��� �������� (400�10���� = 4 ��� - �������� ����� ������ ��� ����� � boot)
	}
	send_boot(); // �������� ����������� (���� ������� ����)

// ��������������, ���� �� ���� �� ����� boot �� ������!
#elif defined(START_BOOTICE)
#warning "BOOTICE mode - no startup-condition"

#else
#error "Select START_ condition for bootloader in main.c"
#endif

	for(;;)  // --- ������ ���������� ---
	{
		val = recvchar();
		// ���� ������ - Autoincrement ��������������?
		if (val == 'a') 
		{
			sendchar('Y');			// ����� - ��, ������� Autoincrement

		// ���� ������ - ������ ������
		} 
		else if (val == 'A') 
		{
			address = recvchar();		//������ ������� 8 MSB ������
			address = (address<<8) | recvchar();
			sendchar('\r');

		// ���� ������ - ��������� ������ ��������
		} 
		else if (val == 'b') 
		{
			sendchar('Y');					// ����� ������ - �������� ��������������
			sendchar((sizeof(gBuffer) >> 8) & 0xFF);	// ����� ������ - ������ � ������
			sendchar(sizeof(gBuffer) & 0xFF);

		// ���� ������ - ����� �������� ������
		} 
		else if (val == 'B') 
		{
			pagebuf_t size;
			size = recvchar() << 8;			// �������� �������� ����� � buffersize
			size |= recvchar();				// �������� �������� ����� � buffersize
			val = recvchar();				// �������� ���� ������ ('E' ��� 'F')
			recvBuffer(size);

			if (device == DEVTYPE) 
			{
				if (val == 'F') 			// ���� ��� ������ flash
				{
				address = writeFlashPage(address, size);	// �������� � Flash
				} 
				else if (val == 'E') 		// ���� ��� ������ eeprom
				{
				address = writeEEpromPage(address, size);	// �������� � EEPROM
				}
				sendchar('\r');
			} 
			else 
			{
			sendchar(0);	// �������� 0 ����
			}

		// ���� ������ - ������ ����� ������
		} 
		else if (val == 'g') 
		{
			pagebuf_t size;
			size = recvchar() << 8;			// �������� �������� ����� � buffersize
			size |= recvchar();				// �������� �������� ����� � buffersize
			val = recvchar();				// �������� ��� ������ ('E' ��� 'F')

			if (val == 'F') 				// ���� ��� ������ flash
			{
			address = readFlashPage(address, size);		// ������ �� Flash
			} 
			else if (val == 'E') 			// ���� ��� ������ eeprom
			{
			address = readEEpromPage(address, size);	// ������ �� EEPROM
			}

		// ���� ������ - ������� ���
 		} 
		else if (val == 'e') 
		{
		if (device == DEVTYPE) 
			{
			eraseFlash(); 	// ������� Flash (���������������� ���������)
			}
		sendchar('\r');

		// ���� ������ - ����� �� ����������
		} 
		else if (val == 'E') 
		{
		wdt_enable(EXIT_WDT_TIME); // ������������ ������� ������ ��� �������� � ����������� ������
		sendchar('\r');				// �������� ������

		//���� ������ - ����� � ����� ����������������
		} 
		else if (val == 'P') 
		{
		sendchar('\r');

		//���� ������ - ����� �� ������ ����������������
		} 
		else if (val == 'L') 
		{
		sendchar('\r');
		//���� ������ - ���� �������������
		} 
		else if (val == 'p') 
		{
		sendchar('S');		// ������ - serial ������������

		//���� ������ - ��� ����������
		} 
		else if (val == 't') 
		{
		sendchar(DEVTYPE);		// �������� ��� ���-�� (Part-Code Boot (0x73))
		sendchar(0);
		// clear and set LED ignored
		} 
		else if ((val == 'x') || (val == 'y')) 
		{
		recvchar();
		sendchar('\r');

		// set device
		} 
		else if (val == 'T') 
		{
		device = recvchar();
		sendchar('\r');
		// ���� ������ - ������� software identifier
		} 
		else if (val == 'S') 
		{
		send_boot();

		//���� ������ - Software Version
		} 
		else if (val == 'V') {
		sendchar(VERSION_HIGH);
		sendchar(VERSION_LOW);

		// ���������� ��������� ���� (�.�.
		// AVRProg ��������� ������� "Atmel-byte" 0x1E ��������� ������
		// ��� ����� ������������ � ���������� ���� window ����������)
		// � ����� ��� �������������!
		} 
		else if (val == 's') 
		{
		sendchar(SIG_BYTE3);
		sendchar(SIG_BYTE2);
		sendchar(SIG_BYTE1);

		/* ESC */
		} 
		else if(val != 0x1b) 
		{
		sendchar('?');
		}

	} 
	return 0;
}
//******************************************** END ***********************************************************************************


