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
* size     : depends on features and startup ( минимальный размер < 512 слов)
* by       : Переделка загрузчика от M. Thomas (2008)
*
****************************************************************************
*	Программа загрузчика v02 для контроллеров КМ (ATMEGA324P)
* Надо в свойствах проекта указать используемый внешний Makefile!!!
*  Открыть makefile и прочитать, там указаны параметры линковщика.  
*  тамже выбрать размер Boot Size (BOOTSIZE=xxxx) и тип CPU
*  Остальные конфигурации выставить в этом файле.
*
*  программа имеет минимальный размер 512 слов это (1024, 0x400 байтов) 
*  т.е. на размер bootloader-section необходимо отводить 1024 байта. 
*
* Написана для контроллеров SEM КМ01-8600.М и КМ01-2442.М - работает!!!
*
* ------------- Прошивка фъюсов --------------------------------------------------
* Установить BOOTCZ = 11 (1k boot) и BOOTRST = 1 (при старте, начало проги с boot)
* Здесь надо выбирать объём памяти загрузчика в 2 раза больше чем в Makefile!!!
****************************************************************************/


/*  Тактовая частота */
#ifndef F_CPU
//#define F_CPU 7372800
#define F_CPU 14745600
#endif

/* UART Скорость UART оптимально 19200 */
//#define BAUDRATE 9600
#define BAUDRATE 19200

/***************************************************
* Тип устройства:
*   Для AVRProg выбирать BOOT 
*   Это корректное значение для bootloader.
*   avrdude может определить только part-code для ISP 
****************************************************/
#define DEVTYPE     DEVTYPE_BOOT
// #define DEVTYPE     DEVTYPE_ISP // это не использую

/***************************************************
 * Выбор порта для джампера входа в загрузчик
 * Чтобы войти в загрузчик надо чтобы при запуске этот джампер был замкнут на +5Вольт
 * Это 1 и 2 пин разъема ISP!
 ***************************************************/
 
#define BLPORT		PORTB
#define BLDDR		DDRB
#define BLPIN		PINB
#define BLPNUM		PINB6

/***************************************************
 * Выбор порта для индикатора работы загрузчика
 * Светодиод горит - мы в загрузчике (использую всегда)
****************************************************/

//#define ENABLE_BOOT_LED
#define BIPORT		PORTB
#define BIDDR		DDRB
#define BIPIN		PINB
#define BIPNUM		PINB3

/****************************************************
* Выбор порта вывода управления передачей
* по RS485 (использую всегда)
*****************************************************/

//#define RS485
#define RSPORT	PORTD
#define RSDDR	DDRD
#define RSPNUM	PIND2

/****************************************************
 * Выключить Собачий таймер на время загрузчика
*****************************************************/
#define DISABLE_WDT_AT_STARTUP

/****************************************************
 * Watchdog-reset is issued at exit 
 * define the timeout-value here (see avr-libc manual)
*****************************************************/
#define EXIT_WDT_TIME   WDTO_250MS

/****************************************************
 * Выбор режима загрузчика
 * SIMPLE-Mode - Загрузчик стартует когда на джампер подано +5В
 *   переход к основной программе осуществляется после сброса 
 *   (джампер должн быть разомкнут) либо по команде от программатора
 *   При этом режиме вывод на джампер конфигурируется как вход-с без подтяга,
 *   но при выходе из загрузчика все выставляется по умолчанию
 * BOOTICE-Mode - для зашивки  JTAGICE файла upgrade.ebn в Мегу16.
 *   что превращает ее в JTAG отладчик. Разумеется нужно добавить весь необходимый
 *   обвяз на кристалл для этого. И частота должна быть везде прописана как 7372800
 *   в F_CPU Для совместимости с родной прошивкой JTAG ICE
 * WAIT-mode Bootloader ожидает команды на вход, если ее не было в течении промежутка времени
 *   (который настраивается) то происходит переход к основной программе.
 *****************************************************/
//#define START_SIMPLE
#define START_WAIT
//#define START_BOOTICE

/* Команда для входа в загрузчик в START_WAIT */
#define START_WAIT_UARTCHAR 'S'

/* Выдержка для START_WAIT mode ( t = WAIT_TIME * 10ms ) */
#define WAIT_VALUE 400 /* сейчас: 400*10ms = 3000ms = 3sec */

/******************************************************************
* Запрет чтения программы bootloader для самой программы bootloader
* запрет на чтение flash (читаются пустые значения (0xFFFF))
*******************************************************************/
//#define READ_PROTECT_BOOTLOADER

#define VERSION_HIGH '0'
#define VERSION_LOW  '8'

#define GET_LOCK_BITS           0x0001
#define GET_LOW_FUSE_BITS       0x0000
#define GET_HIGH_FUSE_BITS      0x0003
#define GET_EXTENDED_FUSE_BITS  0x0002

/* Расчет делителя частоты для USART*/
#define UART_CALC_BAUDRATE(baudRate) ((uint32_t)((F_CPU) + ((uint32_t)baudRate * 8UL)) / ((uint32_t)(baudRate) * 16UL) - 1)

/* Включаемые файлы */

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
	RSPORT |= (1<<RSPNUM);							// 1 на выводе TXE
	UART_DATA = data;								// передать байт в буфер
	while (!(UART_STATUS & (1<<UART_TXREADY))); 	// Ждать конца передачи
	UART_STATUS |= (1<<UART_TXREADY);				// Сбросить флаг т.к. нет прерываний
	_delay_us(500);									// Задержка для передачи последнего байта (по другому не работает!)
	RSPORT &= ~(1<<RSPNUM);						// 0 на выводе TXE
}

static uint8_t recvchar(void)
{
	while (!(UART_STATUS & (1<<UART_RXREADY))); 	// Ждать символ
	return UART_DATA;								// Вернуться с принятым символом
}

static inline void eraseFlash(void)
{
	// стирание только основной программы (bootloader защищен (не стирается))
	uint32_t addr = 0;
	while (APP_END > addr) 
		{
		boot_page_erase(addr);		// Выполняет стирание страницы
		boot_spm_busy_wait();		// Ждать конца стирания страницы.
		addr += SPM_PAGESIZE;		// перейти к следующей странице
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
		boot_page_fill(baddr, data);	// вызов asm подпрограммы.

		baddr += 2;			// выбрать следующее слово в памяти
		size -= 2;			// уменьшить указатель кол-ва байтов для записи
		} 
	while (size);				// выполнять пока все не будет записано

	boot_page_write(pagestart);
	boot_spm_busy_wait();
	boot_rww_enable();		// перейти в RWW секцию

	return baddr>>1;
}

static inline uint16_t writeEEpromPage(uint16_t address, pagebuf_t size)
{
	uint8_t *tmp = gBuffer;

	do 
		{
		eeprom_write_byte( (uint8_t*)address, *tmp++ );
		address++;			// выбрать следующий байт
		size--;				// уменьшить указатель кол-ва байтов для записи
		}
	while (size);				// выполнять пока все не будет записано

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
		// если запрещено чтение bootloader
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
		data = 0xFFFF; // вставлять пустышки
		}
#endif
		sendchar(data);				// передать младший байт LSB 
		sendchar((data >> 8));		// передать старший байт MSB
		baddr += 2;					// выбрать следующее слово в памяти (2 байта)
		size -= 2;					// вычесть два байта из общего числа читаемых байт
	} 
	while (size);					// Повторять пока блок не будет прочитан
	return baddr>>1;
}

static inline uint16_t readEEpromPage(uint16_t address, pagebuf_t size)
{
	do 
	{
	sendchar( eeprom_read_byte( (uint8_t*)address ) );
	address++;
	size--;						// уменьшить кол-во считываемых байт
	} 
	while (size);				// Повторять пока блок не будет прочитан

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


	BIPORT |= (1<<BIPNUM);	// светодиод включить - LED ON (1)
	BIDDR  |= (1<<BIPNUM);

#ifdef DISABLE_WDT_AT_STARTUP // если надо Выключить Собачий таймер на время загрузчика
	#ifdef WDT_OFF_SPECIAL
		#warning "using target specific watchdog_off"
		bootloader_wdt_off();
	#else
		cli();
		wdt_reset();
		wdt_disable(); // Выключить таймер
	#endif
#endif
	
		// Выполнять всегда при запуске
#ifdef START_SIMPLE
	BLDDR  &= ~(1<<BLPNUM);	// Установить вывод кнопки как вход (если загрузчик запускается от джампера)
	BLPORT |= (1<<BLPNUM);		// Подтянуть вход к +
#endif
	// Настроить вывод управления TXE для RS485
	RSPORT &= ~(1<<RSPNUM); 	// на выходе установить 0 (TXE - не активно)
	RSDDR |= (1<<RSPNUM);		// вывод как выход	
	// Установить скорость порта UART
	UART_BAUD_HIGH = (UART_CALC_BAUDRATE(BAUDRATE)>>8) & 0xFF;
	UART_BAUD_LOW = (UART_CALC_BAUDRATE(BAUDRATE) & 0xFF);

	UART_CTRL = UART_CTRL_DATA;		// Включить RX, TX
	UART_CTRL2 = UART_CTRL2_DATA;	// настроить параметры обмена 8,n,1
	

#if defined(START_SIMPLE) // Если работа от джампера !!!!!!!!!!!!!!!!!!!!!!! (мой случай)

	if ((BLPIN & (1<<BLPNUM))) {
		// Перейти к основному приложению если на входе кнопки не 0
		BLPORT &= ~(1<<BLPNUM);	// Установить по умолчанию (не обязательно)

		BIPORT &= ~(1<<BIPNUM);	// Выключить светодиод и установить вывод по умолчанию
		BIDDR  &= ~(1<<BIPNUM);

		RSPORT &= ~(1<<RSPNUM);	// Вывод TXE как вход, по умолчанию
		RSDDR  &= ~(1<<RSPNUM);	

		jump_to_app();				// Перейти в секцию приложения
	}

#elif defined(START_WAIT)	// Если работа по времени!!!!!!!!!!!!!!!!!!!!!! (мой случай)

	uint16_t cnt = 0;

	while (1) {
		if (UART_STATUS & (1<<UART_RXREADY))
			if (UART_DATA == START_WAIT_UARTCHAR)
				break;

		if (cnt++ >= WAIT_VALUE) {		// Если время ожидания вышло!
			BLPORT &= ~(1<<BLPNUM);	// установить все по умолчанию

			BIPORT &= ~(1<<BIPNUM);	// Выключить светодиод и установить вывод по умолчанию
			BIDDR  &= ~(1<<BIPNUM);

			jump_to_app();			// Перейти в секцию приложения
		}

		_delay_ms(10);					// задержка для счетчика (400Х10мсек = 4 сек - ожидания после старта для входа в boot)
	}
	send_boot(); // Передать приглашение (если конечно надо)

// Предупреждение, если ни один из типов boot не выбран!
#elif defined(START_BOOTICE)
#warning "BOOTICE mode - no startup-condition"

#else
#error "Select START_ condition for bootloader in main.c"
#endif

	for(;;)  // --- Работа загрузчика ---
	{
		val = recvchar();
		// если запрос - Autoincrement поддерживается?
		if (val == 'a') 
		{
			sendchar('Y');			// ответ - да, быстрый Autoincrement

		// если запрос - запись адреса
		} 
		else if (val == 'A') 
		{
			address = recvchar();		//чтение старших 8 MSB адреса
			address = (address<<8) | recvchar();
			sendchar('\r');

		// если запрос - поддержка буфера загрузки
		} 
		else if (val == 'b') 
		{
			sendchar('Y');					// ответ буфера - загрузка поддерживается
			sendchar((sizeof(gBuffer) >> 8) & 0xFF);	// ответ буфера - размер в байтах
			sendchar(sizeof(gBuffer) & 0xFF);

		// если запрос - Старт загрузки буфера
		} 
		else if (val == 'B') 
		{
			pagebuf_t size;
			size = recvchar() << 8;			// загрузка старшего байта в buffersize
			size |= recvchar();				// загрузка младшего байта в buffersize
			val = recvchar();				// загрузка типа памяти ('E' или 'F')
			recvBuffer(size);

			if (device == DEVTYPE) 
			{
				if (val == 'F') 			// если это память flash
				{
				address = writeFlashPage(address, size);	// записать в Flash
				} 
				else if (val == 'E') 		// если это память eeprom
				{
				address = writeEEpromPage(address, size);	// записать в EEPROM
				}
				sendchar('\r');
			} 
			else 
			{
			sendchar(0);	// передать 0 байт
			}

		// если запрос - Чтение блока памяти
		} 
		else if (val == 'g') 
		{
			pagebuf_t size;
			size = recvchar() << 8;			// загрузка старшего байта в buffersize
			size |= recvchar();				// загрузка младшего байта в buffersize
			val = recvchar();				// получить тип памяти ('E' или 'F')

			if (val == 'F') 				// если это память flash
			{
			address = readFlashPage(address, size);		// читать из Flash
			} 
			else if (val == 'E') 			// если это память eeprom
			{
			address = readEEpromPage(address, size);	// читать из EEPROM
			}

		// если запрос - Стереть чип
 		} 
		else if (val == 'e') 
		{
		if (device == DEVTYPE) 
			{
			eraseFlash(); 	// Стереть Flash (пользовательскую программу)
			}
		sendchar('\r');

		// если запрос - выйти из обновления
		} 
		else if (val == 'E') 
		{
		wdt_enable(EXIT_WDT_TIME); // активировать собачий таймер для перехода к аппаратному сбросу
		sendchar('\r');				// передать символ

		//если запрос - Войти в режим программирования
		} 
		else if (val == 'P') 
		{
		sendchar('\r');

		//если запрос - выйти из режима программирования
		} 
		else if (val == 'L') 
		{
		sendchar('\r');
		//если запрос - типа программатора
		} 
		else if (val == 'p') 
		{
		sendchar('S');		// всегда - serial программатор

		//если запрос - тип устройства
		} 
		else if (val == 't') 
		{
		sendchar(DEVTYPE);		// передать тип уст-ва (Part-Code Boot (0x73))
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
		// если запрос - вернуть software identifier
		} 
		else if (val == 'S') 
		{
		send_boot();

		//если запрос - Software Version
		} 
		else if (val == 'V') {
		sendchar(VERSION_HIGH);
		sendchar(VERSION_LOW);

		// возвращаем несколько байт (т.к.
		// AVRProg программа ожидает "Atmel-byte" 0x1E последним байтом
		// эти байты показываются в диалоговом окне window приложения)
		// В общем для совместимости!
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


