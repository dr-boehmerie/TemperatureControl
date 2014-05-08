/*
 * Author: Michael Böhme
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

// Temperature Controller

#include <inttypes.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <compat/twi.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>


/*---------------------------Konstanten--------------------------------------*/

#if 1
// 8 MHz / 256 = 31250 Hz; 31250 Hz / 125 = 250 Hz; 250 Hz / 3 = 83 1/3 Hz
#define TIMER0_PRESCALER		(_BV(CS02))
#define TIMER0_OCRA				125
#else
// 8 MHz / 256 = 31250 Hz; 31250 Hz / 50 = 625 Hz
#define TIMER0_PRESCALER		(_BV(CS02))
#define TIMER0_OCRA				50
#endif

#define TIMER0_START			TCCR0B |=  TIMER0_PRESCALER
#define TIMER0_STOP				TCCR0B &= ~TIMER0_PRESCALER

// 8 MHz / 8 = 1 MHz; 1MHz / 65536 = 15.26 Hz
#define TIMER1_PRESCALER		(_BV(CS10))

#define TIMER1_START			TCCR1B |= TIMER1_PRESCALER
#define TIMER1_STOP				TCCR1B &= ~TIMER1_PRESCALER

// 8 MHz / 256 = 31250 Hz; 31250 Hz / (75 * 8) Hz = 52.1
#define TIMER2_PRESCALER		(_BV(CS22) | _BV(CS21))
#define TIMER2_OCRA				50

#define TIMER2_START			TCCR2B |=  TIMER2_PRESCALER
#define TIMER2_STOP				TCCR2B &= ~TIMER2_PRESCALER



// Dallas 1-Wire Bus
#define ONE_WIRE_ENABLE		1

// Anzahl der unterstützten Geräte
#define ONE_WIRE_DEV_NO		2

// Kommandos für Dallas 1-Wire
#define ONE_WIRE_CMD_SRCH_ROM	0xF0
#define ONE_WIRE_CMD_READ_ROM	0x33
#define ONE_WIRE_CMD_MATCH_ROM	0x55
#define ONE_WIRE_CMD_SKIP_ROM	0xCC
#define ONE_WIRE_CMD_ALRM_SRCH	0xEC

// Kommandos für DS18B20
#define ONE_WIRE_CMD_CONVERT_T	0x44
#define ONE_WIRE_CMD_WR_SCRATCH	0x4E
#define ONE_WIRE_CMD_RD_SCRATCH	0xBE
#define ONE_WIRE_CMD_CP_SCRATCH	0x48
#define ONE_WIRE_CMD_RECALL_EE	0xB8
#define ONE_WIRE_RD_SUPPLY		0xB4

// Polynom für die CRC-Berechnung
#define CRC_1WIRE_POLY		0b00110001



// ADC
// Reference Selection
#define ADMUX_REFSEL		0

// Quellen 0 bis 2
#define ADMUX_MIN			0
#define ADMUX_MAX			2

#define ADC_SRC_NO			(ADMUX_MAX - ADMUX_MIN + 1)

#define TEMP_AVERAGE_NO		16

#if 1
// ADC wird mit 250Hz getriggert, 3 Quellen
// Aktualisierungsfrequenz 1 Hz
#define TEMP_UPDATE_COUNT	((250 / 3) * 1)
// Blinkfrequenz 3 Hz
#define TEMP_FLASH_COUNT	((250 / 3) / 3)
// Sekundenzähler: 250 / 3 = 83 1/3
#define TEMP_HIST_COUNT		((250 / 3) * 1)

#else
// ADC wird mit 625Hz getriggert, 3 Quellen
// Aktualisierungsfrequenz 1 Hz
#define TEMP_UPDATE_COUNT	((625 / 3) * 1)
// Blinkfrequenz 3 Hz
#define TEMP_FLASH_COUNT	((625 / 3) / 3)
// Sekundenzähler: 625 / 3 = 208 1/3
#define TEMP_HIST_COUNT		((625 / 3) * 1)
#endif

// solange muss der Ausgangswert konstant bleiben, bevor das Relais umgeschalten wird
#define TEMP_OUTPUT_1_COUNT		30

// zweite Verzögerung für die Kopplung von Kanal 2 an Kanal 1
#define TEMP_OUTPUT_2_COUNT		60


// ADC-Werte für Tastendruck
#define ADC_KEY_MENU_MIN	(0x200 - 0x30)	// diese Taste wackelt ziemlich stark
#define ADC_KEY_MENU_MAX	(0x200 + 0x30)

#define ADC_KEY_UP_MIN		(0x2AA - 0x10)
#define ADC_KEY_UP_MAX		(0x2AA + 0x10)

#define ADC_KEY_DOWN_MIN	(0x300 - 0x10)
#define ADC_KEY_DOWN_MAX	(0x300 + 0x10)

#define ADC_KEY_OK_MIN		(0x334 - 0x10)
#define ADC_KEY_OK_MAX		(0x334 + 0x10)

// wie oft muss eine Taste hintereinander gesampled werden, damit sie gültig ist
#if 0
// (1 / (625 / 3)) * (20 + 1) = 100.8ms
#define MENU_KEY_CNT_MIN	20
#else
// (1 / (250 / 3)) * (8 + 1) = 100.8ms
#define MENU_KEY_CNT_MIN	8
#endif


#define MENU_KEY_MENU		_BV(0)
#define MENU_KEY_UP			_BV(1)
#define MENU_KEY_DOWN		_BV(2)
#define MENU_KEY_OK			_BV(3)


// Grenzwerte für die Parameter
// CH1 hängt an Temp1 oder Temp2
#define TEMP_CFG_CH1_MAX	100
#define TEMP_CFG_CH1_MIN	-10

// CH2 hängt an der Differenz aus den Temperaturen
#define TEMP_CFG_CH2_MAX	100
#define TEMP_CFG_CH2_MIN	0


// Quellenauswahl für den Ausgang
#define TEMP_SRC_0			0
#define TEMP_SRC_1			1
#define TEMP_SRC_0_OR_1		2
#define TEMP_SRC_DELTA		3

// Temp1 oder Temp2
#define OUTPUT_CH1_SRC		TEMP_SRC_0_OR_1

// absolute Differenz von Temp1 und Temp2 als Quelle
#define OUTPUT_CH2_SRC		TEMP_SRC_DELTA

// so spart man sich eine Tabelle
#define OUTPUT_CHx_SRC(CH)	(CH == 0 ? OUTPUT_CH1_SRC : OUTPUT_CH2_SRC)



// Speicherung der Parameter im EEPROM
#define TEMP_CFG_EE_COUNT	4
#define TEMP_CFG_EE_OFFSET	0

#if 0

#define temp_val_t		int8_t
#define TEMP_VAL_MAX	INT8_MAX
#define TEMP_VAL_MIN	INT8_MIN

#else

#define temp_val_t		int16_t
#define TEMP_VAL_MAX	INT16_MAX
#define TEMP_VAL_MIN	INT16_MIN

#endif

/*---------------------------Aliase für Pins und Ports-----------------------*/

#define DIGIT_NO			8

// hängt alles schön der Reihe nach an PortD
#define SEGMENT_A			_BV(PD0)
#define SEGMENT_B			_BV(PD1)
#define SEGMENT_C			_BV(PD2)
#define SEGMENT_D			_BV(PD3)
#define SEGMENT_E			_BV(PD4)
#define SEGMENT_F			_BV(PD5)
#define SEGMENT_G			_BV(PD6)
#define SEGMENT_DP			_BV(PD7)

// Ausgänge
#define OUTPUT_CHx_REG		PORTC
#define OUTPUT_CH1_BIT		_BV(PC4)
#define OUTPUT_CH2_BIT		_BV(PC5)
#define OUTPUT_CHx_MASK		(OUTPUT_CH1_BIT | OUTPUT_CH2_BIT)

//#define OUTPUT_CH1_ON		OUTPUT_CHx_REG |=  OUTPUT_CH1_BIT
//#define OUTPUT_CH1_OFF		OUTPUT_CHx_REG &= ~OUTPUT_CH1_BIT

//#define OUTPUT_CH2_ON		OUTPUT_CHx_REG |=  OUTPUT_CH2_BIT
//#define OUTPUT_CH2_OFF		OUTPUT_CHx_REG &= ~OUTPUT_CH2_BIT

// Tabelle gespart
#define OUTPUT_CHx_BIT(CH)	(CH == 0 ? OUTPUT_CH1_BIT : OUTPUT_CH2_BIT)



#if ONE_WIRE_ENABLE

// PortPin auf 0 und als Ausgang
#define ONE_WIRE_OUT_LO		{PORTC &= ~_BV(PC3); DDRC |= _BV(PC3);}

// PortPin als Ausgang, die 0 wird nur einmal im oneWire_reset ins PORT-Register geschrieben
#define ONE_WIRE_OUT		{DDRC |= _BV(PC3);}

// PortPin als Eingang
#define ONE_WIRE_RELEASE	{DDRC &= ~_BV(PC3);}

// Wert des PortPins lesen
#define ONE_WIRE_READ		(PINC & _BV(PC3))

// Standard-Verzögerungen laut AppNote 126
#define ONE_WIRE_DELAY_A			_delay_us(6)
#define ONE_WIRE_DELAY_B			_delay_us(64)
#define ONE_WIRE_DELAY_C			_delay_us(60)
#define ONE_WIRE_DELAY_D			_delay_us(10)
#define ONE_WIRE_DELAY_E			_delay_us(9)
#define ONE_WIRE_DELAY_F			_delay_us(55)
#define ONE_WIRE_DELAY_G			(void)0			//_delay_us(0) bei 0 wird trotzdem 1 tick gewartet, das muss nicht sein
#define ONE_WIRE_DELAY_H			_delay_us(480)
#define ONE_WIRE_DELAY_I			_delay_us(70)
#define ONE_WIRE_DELAY_J			_delay_us(410)

#endif

/*---------------------------Variablen---------------------------------------*/

struct display_data
{
	volatile uint8_t	digit;			/*!< aktive Stelle */
	uint8_t				mem[DIGIT_NO];	/*!< Zeichenspeicher */
	uint8_t				seg[DIGIT_NO];	/*!< Segmentspeicher */
} dspl;


/*
	 --		 A
	|  |	F B
	 --		 G
	|  |	E C
	 --		 D

	 .	|	D A
	---		C
	 '	|	E B
*/

const uint8_t digits[] PROGMEM =
{
	/*  0 */	(SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F),
	/*  1 */	(SEGMENT_B | SEGMENT_C),
	/*  2 */	(SEGMENT_A | SEGMENT_B | SEGMENT_G | SEGMENT_E | SEGMENT_D),
	/*  3 */	(SEGMENT_A | SEGMENT_B | SEGMENT_G | SEGMENT_C | SEGMENT_D),
	/*  4 */	(SEGMENT_F | SEGMENT_G | SEGMENT_B | SEGMENT_C),
	/*  5 */	(SEGMENT_A | SEGMENT_F | SEGMENT_G | SEGMENT_C | SEGMENT_D),
	/*  6 */	(SEGMENT_A | SEGMENT_F | SEGMENT_E | SEGMENT_D | SEGMENT_C | SEGMENT_G),
	/*  7 */	(SEGMENT_A | SEGMENT_B | SEGMENT_C),
	/*  8 */	(SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F | SEGMENT_G),
	/*  9 */	(SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_F | SEGMENT_G),

	/*  A */	(SEGMENT_E | SEGMENT_F | SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_G),
	/*  b */	(SEGMENT_F | SEGMENT_E | SEGMENT_D | SEGMENT_C | SEGMENT_G),
	/*  c */	(SEGMENT_G | SEGMENT_E | SEGMENT_D),
	/*  d */	(SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_G),
	/*  E */	(SEGMENT_A | SEGMENT_F | SEGMENT_E | SEGMENT_D | SEGMENT_G),
	/*  F */	(SEGMENT_E | SEGMENT_F | SEGMENT_A | SEGMENT_G),

	/* 0_ */	(0),
	/* 1_ */	(SEGMENT_A | SEGMENT_B),
	/* P0 */	(SEGMENT_D | SEGMENT_C | SEGMENT_E),
	/* M0 */	(SEGMENT_C),
	/* P1 */	(SEGMENT_D | SEGMENT_C | SEGMENT_E | SEGMENT_A | SEGMENT_B),
	/* M1 */	(SEGMENT_C | SEGMENT_A | SEGMENT_B),

	/* PU */	(SEGMENT_C | SEGMENT_D),
	/* PD */	(SEGMENT_C | SEGMENT_E),

	/* MINUS */	(SEGMENT_G),

	/*  H */	(SEGMENT_E | SEGMENT_F | SEGMENT_G | SEGMENT_B | SEGMENT_C),
	/*  I */	(SEGMENT_E | SEGMENT_F),
	/*  L */	(SEGMENT_E | SEGMENT_F | SEGMENT_D),
	/* L_ */	(SEGMENT_B | SEGMENT_C | SEGMENT_D),
	/*  N */	(SEGMENT_E | SEGMENT_F | SEGMENT_A | SEGMENT_B | SEGMENT_C),
	/*  R */	(SEGMENT_E | SEGMENT_A | SEGMENT_F | SEGMENT_G | SEGMENT_C | SEGMENT_D),
	/*  S */	(SEGMENT_A | SEGMENT_F | SEGMENT_G | SEGMENT_C | SEGMENT_D),
	/*  U */	(SEGMENT_E | SEGMENT_F | SEGMENT_D | SEGMENT_B | SEGMENT_C),

	/* Blank */	(0),

};

enum digit_list
{
	DIGIT_0,
	DIGIT_1,
	DIGIT_2,
	DIGIT_3,
	DIGIT_4,
	DIGIT_5,
	DIGIT_6,
	DIGIT_7,
	DIGIT_8,
	DIGIT_9,

	DIGIT_A,
	DIGIT_B,
	DIGIT_C,
	DIGIT_D,
	DIGIT_E,
	DIGIT_F,

	DIGIT_0_,
	DIGIT_1_,
	DIGIT_P0,
	DIGIT_M0,
	DIGIT_P1,
	DIGIT_M1,

	DIGIT_PU,
	DIGIT_PD,

	DIGIT_MINUS,

	DIGIT_H,
	DIGIT_I,
	DIGIT_L,
	DIGIT_L_,
	DIGIT_N,
	DIGIT_R,
	DIGIT_S,
	DIGIT_U,

	DIGIT_BLANK,

};


struct adc_data_s
{
	uint8_t		source;				/*!< Quelle */
	uint8_t		complete;			/*!< wird nach Erfassung aller Quellen 1 */

	uint16_t	mem[ADC_SRC_NO];	/*!< Speicher für die aktuellen Ergebnisse */
};

// alles wird auch in der ISR beschrieben
volatile struct adc_data_s		adc_data;

#if ONE_WIRE_ENABLE
// keine Mittelwertbildung mehr
#else
#if TEMP_AVERAGE_NO > 0
struct temp_average_data
{
	uint16_t	sum[2];
	uint16_t	avg[2];
	uint16_t	array[2][TEMP_AVERAGE_NO];

	uint8_t		index;
	uint8_t		count;

} temp_data;
#endif
#endif

struct temp_history
{
	uint8_t		cnt;		/*!< Erfassungszähler */
	uint8_t		seconds;	/*!< Sekundenzähler */
	uint8_t		add_sec;	/*!< Schaltsekunden */
	uint8_t		minutes;	/*!< Minutenzähler */
	uint8_t		hours;		/*!< nur bis 23 */

	uint8_t		index;		/*!< Arrayindex */

	uint8_t		valid[2];	/*!< Messwert ist gültig */

	temp_val_t	value[2];	/*!< aktuelle Werte */

	// Min/Max-Werte der letzten 24 Stunden
	temp_val_t	min_array[2][24];
	temp_val_t	max_array[2][24];

#if 0
	// Min/Max-Werte der letzten 6/12/24 Stunden
	temp_val_t	min[2][3];
	temp_val_t	max[2][3];
#endif

} temp_hist;

struct temp_config_data
{
	uint8_t		counter;	/*!< Schreibzähler an erster Stelle! */

	int8_t		ch1_on;		/*!< Kanal 1 ein */
	int8_t		ch1_off;	/*!< Kanal 1 aus */

	int8_t		ch2_on;		/*!< Kanal 2 ein */
	int8_t		ch2_off;	/*!< Kanal 2 aus */

	uint8_t		reserved[8 - 5];	/*!< Platzhalter */

} temp_ee_cfg;


enum PARA_LIST
{
	CFG_PARA_CH1_ON,
	CFG_PARA_CH2_ON,

	CFG_PARA_CH1_OFF,
	CFG_PARA_CH2_OFF,

	CFG_PARA_END,
	MENU_PARA_START		= CFG_PARA_END,

	MENU_PARA_TEMP		= MENU_PARA_START,

	MENU_PARA_MAX_CH1,
	MENU_PARA_MAX_CH2,

	MENU_PARA_MIN_CH1,
	MENU_PARA_MIN_CH2,


#if 0
	MENU_PARA_SECONDS,
	MENU_PARA_MINUTES,
	MENU_PARA_HOURS,
#endif

	MENU_PARA_END,

	PARA_NO = MENU_PARA_END
};


struct config_data
{
	uint8_t		cfg_id;		/*!< aktuelle EE-Speicher-ID */

	int8_t		para[CFG_PARA_END];

} temp_cfg;


struct menu_data
{
	uint8_t		menu;		/*!< aktives Menü */
	uint8_t		changed;	/*!< Änderungen an Anzeige oder Menü */
	uint8_t		flash;		/*!< wechselt beim Blinken */

	uint8_t		key;		/*!< Tastendruck */
	uint8_t		keyLast;	/*!< vorhergehender Tastenmesswert */
	uint8_t		keyCnt;		/*!< Anzahl identischer Tastenmesswerte */

//	uint8_t		parMenu;	/*!< Parameter Taste 1 */
//	uint8_t		parUp;		/*!< Parameter Taste 2 */
//	uint8_t		parDown;	/*!< Parameter Taste 3 */
//	uint8_t		parOk;		/*!< Parameter Taste 4 */

	uint8_t		minMaxId;	/*!< Index für MinMaxArrays */

	uint8_t		cnt_update;		/*!< Zähler für Aktualisierung */
	uint8_t		cnt_flash;		/*!< Zähler fürs Blinken */
	uint8_t		cnt_output[2];	/*!< Zähler für Ausgabe */

} menu_cfg;

const uint8_t text_tab[][4] PROGMEM =
{
	//	+/- 1/2			1				2				3
	{	DIGIT_BLANK,	DIGIT_BLANK,	DIGIT_BLANK,	DIGIT_BLANK	},

	{	DIGIT_PU,		DIGIT_H,		DIGIT_E,		DIGIT_I		},
	{	DIGIT_PD,		DIGIT_H,		DIGIT_E,		DIGIT_I		},

	{	DIGIT_PU,		DIGIT_L,		DIGIT_U,		DIGIT_E		},
	{	DIGIT_PD,		DIGIT_L,		DIGIT_U,		DIGIT_E		},

	{	DIGIT_BLANK,	DIGIT_S,		DIGIT_N,		DIGIT_0		},

	{	DIGIT_P0,		DIGIT_MINUS,	DIGIT_MINUS,	DIGIT_MINUS	},
	{	DIGIT_M0,		DIGIT_MINUS,	DIGIT_MINUS,	DIGIT_MINUS	},
};

enum TEXT_LIST
{
	TEXT_ID_BLANK,

	TEXT_ID_CH1_ON,
	TEXT_ID_CH1_OFF,

	TEXT_ID_CH2_ON,
	TEXT_ID_CH2_OFF,

	TEXT_ID_ON_WIRE,

	TEXT_ID_OVF_PLUS,
	TEXT_ID_OVF_MINUS,

	TEXT_ID_NO
};


struct menu_setup_s
{
	uint8_t		text_id;			/*!< Text für Zeile 1 */
	uint8_t		menu_key_menu;		/*!< für jede Taste ein Folgemenü */
	uint8_t		menu_key_up;		/*!< für jede Taste ein Folgemenü */
	uint8_t		menu_key_down;		/*!< für jede Taste ein Folgemenü */
	uint8_t		menu_key_ok;		/*!< für jede Taste ein Folgemenü */

	uint8_t		para;				/*!< Parameter für Taste Hoch/Runter */
	uint8_t		para_cmp;			/*!< Parameter für Vergleich */
	int8_t		para_min;			/*!< Grenzwert für aktuellen Parameter */
	int8_t		para_max;			/*!< Grenzwert für aktuellen Parameter */

} menu_setup;

enum MENU_LIST
{
	MENU_TEMP_VALUE,

	MENU_TEMP_MAX_CH1,
	MENU_TEMP_MAX_CH2,

	MENU_TEMP_MIN_CH1,
	MENU_TEMP_MIN_CH2,

#if 0
	MENU_SELECT_HOURS,
	MENU_SELECT_MINUTES,
	MENU_SELECT_SECONDS,
#endif

	MENU_SELECT_CH1_ON,
	MENU_SELECT_CH1_OFF,

	MENU_SELECT_CH2_ON,
	MENU_SELECT_CH2_OFF,


	MENU_EDIT_CH1_ON,
	MENU_EDIT_CH1_OFF,

	MENU_EDIT_CH2_ON,
	MENU_EDIT_CH2_OFF,


	MENU_NO
};

#if 1
#define MENU_SELECT_SECONDS		MENU_SELECT_CH2_OFF
#define MENU_SELECT_HOURS		MENU_SELECT_CH1_ON
#endif

const struct menu_setup_s menu_setup_tab[MENU_NO] PROGMEM =
{
	//							Text				MENU				UP						DOWN					OK						para				para_cmp			para_min			para_max
	/* MENU_TEMP_VALUE */		{TEXT_ID_NO,		MENU_SELECT_CH1_ON,	MENU_TEMP_MAX_CH1,		MENU_TEMP_MIN_CH1,		MENU_NO,				MENU_PARA_TEMP,		},

	/* MENU_TEMP_MAX_CH1 */		{TEXT_ID_NO,		MENU_NO,			MENU_TEMP_MAX_CH2,		MENU_TEMP_VALUE,		MENU_NO,				MENU_PARA_MAX_CH1,	},
	/* MENU_TEMP_MAX_CH2 */		{TEXT_ID_NO,		MENU_NO,			MENU_NO,				MENU_TEMP_MAX_CH1,		MENU_NO,				MENU_PARA_MAX_CH2,	},

	/* MENU_TEMP_MIN_CH1 */		{TEXT_ID_NO,		MENU_NO,			MENU_TEMP_VALUE,		MENU_TEMP_MIN_CH2,		MENU_NO,				MENU_PARA_MIN_CH1,	},
	/* MENU_TEMP_MIN_CH2 */		{TEXT_ID_NO,		MENU_NO,			MENU_TEMP_MIN_CH1,		MENU_NO,				MENU_NO,				MENU_PARA_MIN_CH2,	},

#if 0
	/* MENU_SELECT_HOURS */		{TEXT_ID_CH1_ON,	MENU_TEMP_VALUE,	MENU_SELECT_CH2_OFF,	MENU_SELECT_MINUTES,	MENU_NO,				MENU_PARA_HOURS,	PARA_NO,	},
	/* MENU_SELECT_MINUTES */	{TEXT_ID_CH1_ON,	MENU_TEMP_VALUE,	MENU_SELECT_HOURS,		MENU_SELECT_SECONDS,	MENU_NO,				MENU_PARA_MINUTES,	PARA_NO,	},
	/* MENU_SELECT_SECONDS */	{TEXT_ID_CH1_ON,	MENU_TEMP_VALUE,	MENU_SELECT_MINUTES,	MENU_SELECT_CH1_ON,		MENU_NO,				MENU_PARA_SECONDS,	PARA_NO,	},
#endif

	/* MENU_SELECT_CH1_ON */	{TEXT_ID_CH1_ON,	MENU_TEMP_VALUE,	MENU_SELECT_SECONDS,	MENU_SELECT_CH1_OFF,	MENU_EDIT_CH1_ON,		CFG_PARA_CH1_ON,	PARA_NO,	},
	/* MENU_SELECT_CH1_OFF */	{TEXT_ID_CH1_OFF,	MENU_TEMP_VALUE,	MENU_SELECT_CH1_ON,		MENU_SELECT_CH2_ON,		MENU_EDIT_CH1_OFF,		CFG_PARA_CH1_OFF,	PARA_NO,	},

	/* MENU_SELECT_CH2_ON */	{TEXT_ID_CH2_ON,	MENU_TEMP_VALUE,	MENU_SELECT_CH1_OFF,	MENU_SELECT_CH2_OFF,	MENU_EDIT_CH2_ON,		CFG_PARA_CH2_ON,	PARA_NO,	},
	/* MENU_SELECT_CH2_OFF */	{TEXT_ID_CH2_OFF,	MENU_TEMP_VALUE,	MENU_SELECT_CH2_ON,		MENU_SELECT_HOURS,		MENU_EDIT_CH2_OFF,		CFG_PARA_CH2_OFF,	PARA_NO,	},


	/* MENU_EDIT_CH1_ON */		{TEXT_ID_CH1_ON,	MENU_SELECT_CH1_ON,		MENU_NO,			MENU_NO,				MENU_SELECT_CH1_ON,		CFG_PARA_CH1_ON,	CFG_PARA_CH1_OFF,	TEMP_CFG_CH1_MIN,	TEMP_CFG_CH1_MAX	},
	/* MENU_EDIT_CH1_OFF */		{TEXT_ID_CH1_OFF,	MENU_SELECT_CH1_OFF,	MENU_NO,			MENU_NO,				MENU_SELECT_CH1_OFF,	CFG_PARA_CH1_OFF,	CFG_PARA_CH1_ON,	TEMP_CFG_CH1_MIN,	TEMP_CFG_CH1_MAX	},

	/* MENU_EDIT_CH2_ON */		{TEXT_ID_CH2_ON,	MENU_SELECT_CH2_ON,		MENU_NO,			MENU_NO,				MENU_SELECT_CH2_ON,		CFG_PARA_CH2_ON,	CFG_PARA_CH2_OFF,	TEMP_CFG_CH2_MIN,	TEMP_CFG_CH2_MAX	},
	/* MENU_EDIT_CH2_OFF */		{TEXT_ID_CH2_OFF,	MENU_SELECT_CH2_OFF,	MENU_NO,			MENU_NO,				MENU_SELECT_CH2_OFF,	CFG_PARA_CH2_OFF,	CFG_PARA_CH2_ON,	TEMP_CFG_CH2_MIN,	TEMP_CFG_CH2_MAX	},
};

#if ONE_WIRE_ENABLE
struct oneWire_s
{
	uint8_t		dev_count;		/*!< Auswahl des ID-Speichers */

	uint8_t		last_device;	/*!< 1 wenn letztes Gerät gesucht wurde */
	uint8_t		last_disc;		/*!< Bitposition des letzten Unterschiedes */
//	uint8_t		last_fam_disc;	/*!< Unterschied im Family-Byte */
	uint8_t		crc8;			/*!< CRC8 der ID */

	uint8_t		rom[ONE_WIRE_DEV_NO][8];	/*!< ID-Speicher */

	struct ds18b20_data
	{
		uint8_t	temp_lo;		/*!< Temperatur LSB */
		uint8_t	temp_hi;		/*!< Temperatur MSB */
		uint8_t	th;				/*!< Temp High Schwelle */
		uint8_t	tl;				/*!< Temp Low Schwelle */
		uint8_t	config;			/*!< Konfigurationsregister */
		uint8_t	reserved[3];	/*!< reserviert */
		uint8_t	crc;			/*!< Prüfsumme über 8 Bytes */
	} data;						/*!< Zwischenspeicher für Daten */

} oneWire;
#endif

struct output_data
{
	uint8_t		reg1[2];	/*!< aktuelle Registerzustände nach 1. Verzögerungszeit */
	uint8_t		reg2[2];	/*!< aktuelle Registerzustände nach 2. Verzögerungszeit */

	uint8_t		current[2];		/*!< aktuelle Zustände */

	uint8_t		count[2];		/*!< Zähler für Änderungserkennung */
} output_data;

/*---------------------------ISR-Deklarationen------------------------------*/


/*---------------------------Unterprogramm-Deklarationen---------------------*/
static void periph_init(void); // Peripherie initialisieren


static void dspl_uint8 (uint8_t pos, uint8_t dp, uint8_t value)		__attribute__((__unused__));
static void dspl_hex_uint8 (uint8_t pos, uint8_t value);
static void dspl_int8 (uint8_t pos, uint8_t dp, int8_t value);
static void dspl_int16 (uint8_t pos, uint8_t dp, int16_t value);
static void dspl_hex_uint16 (uint8_t pos, uint16_t value)			__attribute__((__unused__));
static void dspl_text (uint8_t pos, uint8_t txtId);

static void dspl_mem2seg (uint8_t pos);

static void menu_readKey (uint8_t key);
static void menu_printMenu (void);

static int8_t menu_incr (int8_t val, int8_t cmp, int8_t max);
static int8_t menu_decr (int8_t val, int8_t cmp, int8_t min);

static void menu_loadConfig (void);
static void menu_saveConfig (void);

static void temp_startTemp (void);
static void temp_readTemp (void);
static uint8_t temp_incrSeconds (void);
static void temp_updCurMinMax (void);
static void temp_updHistMinMax (void);

static void temp_updOutput (void);

#if ONE_WIRE_ENABLE
static uint8_t oneWire_reset (void);

void oneWire_writeBit (uint8_t data);
void oneWire_writeByte (uint8_t data);

uint8_t oneWire_readBit (void);
uint8_t oneWire_readByte (void);

static uint8_t oneWire_findFirst (void);
static uint8_t oneWire_findNext (void);
static uint8_t oneWire_search (void);

uint8_t oneWire_selectDev (uint8_t dev);

void oneWire_updateCRC (uint8_t byte);

#endif

/*---------------------------Hauptprogramm-----------------------------------*/
int main(void)
{
	uint8_t		m;
	static uint8_t		key;

	// alle Peripherie initialisieren
	periph_init ();

	// 1s Pause
	for (m = 0; m < 100; m++)
		_delay_ms (10);


	// Ziffern initialisieren
	dspl_text (0, TEXT_ID_BLANK);
	dspl_text (1, TEXT_ID_BLANK);

	// LED-Ausgänge ein
	DDRB = 0xFF;
	DDRD = 0xFF;


	// Parameter laden
	menu_loadConfig ();

	// Menü initialisieren
	menu_cfg.menu = MENU_TEMP_VALUE;
	menu_cfg.key = 0xFF;
	menu_cfg.changed = 1;

	// Spitzenwerte initialisieren
	for (uint8_t i = 0; i < 2; i++) {
	//	for (uint8_t j = 0; j < 24; j++) {
		// Spitzenwertarray aktualisieren
			temp_hist.min_array[i][0] = TEMP_VAL_MAX;
			temp_hist.max_array[i][0] = TEMP_VAL_MIN;
	//	}
#if 0
		// Spitzenwerte der letzten 6, 12 und 24 Stunden
		for (uint8_t j = 0; j < 3; j++) {
			temp_hist.min[i][j] = TEMP_VAL_MAX;
			temp_hist.max[i][j] = TEMP_VAL_MIN;
		}
#endif
	}
	temp_hist.index = 0;

	// Interrupte ein
	sei();

	// Display-Timer starten
	TIMER2_START;

#if ONE_WIRE_ENABLE
	// ersten Sensor suchen
	if (oneWire_findFirst() != 0) {
		// weitere Sensoren suchen
		while (oneWire_findNext() != 0) {
			// die Geräteanzahl wird intern inkrementiert
		}
	} else {
		// nichts gefunden
	}

	// gefundene Anzahl kurz anzeigen
	dspl_text (0, TEXT_ID_ON_WIRE);
	dspl_hex_uint8 (1, oneWire.dev_count);

	// 2s Pause
	for (m = 0; m < 200; m++)
		_delay_ms (10);

#if 0
	for (uint8_t dev = 0; dev < oneWire.dev_count; dev++) {
		// alle ID-Bytes nacheinander anzeigen
		for (key = 0; key < 8; key++) {
			dspl_hex_uint8 (0, (dev << 4) | key);
			dspl_hex_uint8 (1, oneWire.rom[dev][key]);

			// 2s Pause
			for (m = 0; m < 200; m++)
				_delay_ms (10);
		}
	}
#endif

#if ONE_WIRE_ENABLE
	if (oneWire.dev_count > 0) {
		// erste Temperaturerfassung starten
		temp_startTemp ();
	}

	// 1s Pause, damit die Erfassung fertig ist
	for (m = 0; m < 100; m++)
		_delay_ms (10);
#endif

#endif	// ONE_WIRE_ENABLE

	// ADC-Timer starten
	TIMER0_START;

	while (1) {

		// warten auf die Erfassung
		if (adc_data.complete != 0) {
			adc_data.complete = 0;

			// Taste kopieren, nur 8Bit für schnelleren Vergleich
			key = adc_data.mem[2] >> 2;
		//	dspl_hex_uint16 (1, adc_data.mem[2]);

#if ONE_WIRE_ENABLE
			// ab sofort vom DS18B20
#else
			// Ergebnisse anzeigen, 10Bit gehen auf alle Fälle
#if TEMP_AVERAGE_NO > 0
			for (uint8_t i = 0; i < 2; i++) {
				// älteste Werte abziehen
				temp_data.sum[i] -= temp_data.array[i][temp_data.index];
				// neue Werte merken
				temp_data.array[i][temp_data.index] = adc_data.mem[i];
				// neue Werte zur Summe addieren
				temp_data.sum[i] += adc_data.mem[i];
				// Mittelwerte, mit etwas höherer Genauigkeit
				temp_data.avg[i] = temp_data.sum[i] / (TEMP_AVERAGE_NO / 4);
			//	temp_data.avg[i] = temp_data.sum[i] / 2;

				// erstmal Genauigkeit wegwerfen
				temp_hist.value[i] = temp_data.avg[i] >> 2;
			}
			// Index dekrementieren
			if (temp_data.index > 0)
				temp_data.index--;
			else
				temp_data.index = TEMP_AVERAGE_NO - 1;

			// Spitzenwerte erst nachdem die Mittelwerte vollgelaufen sind
			if (temp_data.count < TEMP_AVERAGE_NO) {
				temp_data.count++;
			} else
#else
			// erstmal Genauigkeit wegwerfen
			temp_hist.value[0] = adc_data.mem[0] >> 2;
			temp_hist.value[1] = adc_data.mem[1] >> 2;
#endif
			{
				// Spitzenwerte aktualisieren
				for (uint8_t i = 0; i < 2; i++) {
					// erstmal Genauigkeit wegwerfen
					int8_t	tmp = temp_hist.value[i];

					// Spitzenwertarray aktualisieren
					if (temp_hist.min_array[i][temp_hist.index] > tmp)
						temp_hist.min_array[i][temp_hist.index] = tmp;

					if (temp_hist.max_array[i][temp_hist.index] < tmp)
						temp_hist.max_array[i][temp_hist.index] = tmp;

					// Spitzenwerte der letzten 6, 12 und 24 Stunden
					for (uint8_t j = 0; j < 3; j++) {
						if (temp_hist.min[i][j] > tmp)
							temp_hist.min[i][j] = tmp;

						if (temp_hist.max[i][j] < tmp)
							temp_hist.max[i][j] = tmp;
					}
				}
			}
#endif
			// Erfassungen zählen
			if (temp_hist.cnt < (TEMP_HIST_COUNT - 1)) {
				temp_hist.cnt++;

			} else {
				// zurücksetzen
				temp_hist.cnt = 0;

				if (    menu_cfg.menu == MENU_TEMP_VALUE
#if 0
					|| (menu_cfg.menu >= MENU_SELECT_HOURS && menu_cfg.menu <= MENU_SELECT_SECONDS)
#endif
				) {
					// aktuelle Messwerte zyklisch aktualisieren
					menu_cfg.changed = 1;
				}

#if ONE_WIRE_ENABLE
				if (oneWire.dev_count > 0) {
					// Temperaturen im Sekundentakt lesen (die Konvertierungszeit beträgt max 750ms)
					temp_readTemp ();

					// nächste Erfassung starten, Befehl gleich an alle Sensoren senden
					temp_startTemp ();

					// Spitzenwerte aktualisieren
					temp_updCurMinMax ();
#endif

					// Sekunden inkrementieren, liefert 1 wenn eine Stunde voll ist
					if (temp_incrSeconds() != 0) {
						// MinMax-Werte der letzten Stunden aktualisieren
						temp_updHistMinMax ();
					}

					// Messwerte ausgeben, wenn nicht gerade beim Einstellen
					if (   menu_cfg.menu < MENU_EDIT_CH1_ON
						|| menu_cfg.menu > MENU_EDIT_CH2_OFF)
					{
						// Werte vergleichen, Ausgänge schalten
						temp_updOutput ();
					}
#if ONE_WIRE_ENABLE
				}
#endif
			}

			// Erfassungen zählen
			if (menu_cfg.cnt_flash < (TEMP_FLASH_COUNT - 1)) {
				menu_cfg.cnt_flash++;

			} else {
				// zurücksetzen
				menu_cfg.cnt_flash = 0;

				// Blinken beim Einstellen oder wenn aktuelle Werte ungültig sind
				if (menu_cfg.menu == MENU_TEMP_VALUE
					|| (menu_cfg.menu >= MENU_EDIT_CH1_ON
						&& menu_cfg.menu <= MENU_EDIT_CH2_OFF))
				{
					// toggeln
					menu_cfg.flash ^= 1;
					// aktualisieren
					menu_cfg.changed = 1;
				} else {
					// nicht mehr blinken
					if (menu_cfg.flash != 0) {
						menu_cfg.flash = 0;
						menu_cfg.changed = 1;
					}
				}
			}

			// Tastendruckerkennung
			menu_readKey (key);

			// Menü prüfen und anzeigen
			menu_printMenu ();
		}
	}

	return 0;
}


/*---------------------------Unterprogramme----------------------------------*/
void periph_init(void) //Timer0 initialisieren
{
	/*	Timer läuft im CTC-Mode -> WGM02:0 = 010, die Output Compare Ausgänge
		werden nicht benutzt -> COM0A1:0 = 00, COM0B1:0 = 00, als Clock
		erstmal nix, der Timer soll noch nicht loslaufen -> CS02:0 = 000*/
	TCCR0A = _BV(WGM01);
//	TCCR0B = 0;
//	TCNT0 = 0;
	OCR0A = TIMER0_OCRA;
	// der ADC braucht das Interrupt-Flag als Startsignal
	TIMSK0 = _BV(OCIE0A);

	/*	Timer läuft im CTC-Mode -> WGM13:0 = 0100, die Output Compare Ausgänge
		werden nicht benutzt -> COM1A1:0 = 00, COM1B1:0 = 00, als Clock
		erstmal nix, der Timer soll noch nicht loslaufen -> CS12:0 = 000*/
//	TCCR1A = 0;
//	TCCR1B = _BV(WGM12);
//	TCCR1C = 0;
//	TCNT1H = 0;
//	TCNT1L = 0;
//	OCR1AH = 0x7A;
//	OCR1AL = 0x12;
//	TIMSK1 = _BV(OCIE1A);

	/*	Timer läuft im CTC-Mode -> WGM02:0 = 010, die Output Compare Ausgänge
		werden nicht benutzt -> COM0A1:0 = 00, COM0B1:0 = 00, als Clock
		erstmal nix, der Timer soll noch nicht loslaufen -> CS02:0 = 000*/
	TCCR2A = _BV(WGM21);
//	TCCR2B = 0;
//	TCNT2 = 0;
	OCR2A = TIMER2_OCRA;
	TIMSK2 = _BV(OCIE2A);

	/* ADC */
	adc_data.source = ADMUX_MIN;
	ADMUX = ADMUX_MIN | ADMUX_REFSEL;
	// ADC Trigger Source: Timer0 Compare Match A
	ADCSRB = _BV(ADTS1) | _BV(ADTS0);
	// ADC ein und Prescaler auf 64, Interrupt ein, Auto Trigger enable
	ADCSRA = _BV(ADEN) | _BV(ADIE) | _BV(ADATE) | _BV(ADPS2) | _BV(ADPS1);

	/* Power Reduction Register */
	PRR = 0
	//	| _BV(PRADC)	// ADC aus
		| _BV(PRUSART0)	// UART aus
		| _BV(PRSPI)	// SPI aus
		| _BV(PRTWI)	// TWI aus
	//	| _BV(PRTIM0)	// Timer0 aus
		| _BV(PRTIM1)	// Timer1 aus
	//	| _BV(PRTIM2)	// Timer2 aus
		;

	/* Digital I/O Disable Register */
	DIDR0 = _BV(ADC0D) | _BV(ADC1D) | _BV(ADC2D);	// | _BV(ADC3D);

#if I2C_ENABLE
	/* TWI */
	/* initialize TWI clock: 100 kHz clock, TWPS = 0 => prescaler = 1 */
	#if defined TWPS0
		/* has prescaler (mega128 & newer) */
		TWSR = 0;
	#endif

	#if F_CPU < 3600000UL
		TWBR = 10;			/* smallest TWBR value, see note [5] */
	#else
		TWBR = (F_CPU / 100000UL - 16) / 2;
	#endif

	// TWI ein
	TWCR = _BV(TWEN);
#endif	// I2C_ENABLE

#if 0
	/* SPI */
	// enable SPI Interrupt and SPI in Master Mode with SCK = CK/128
#ifdef SPI_INTERRUPT_DRIVEN
	SPCR = _BV(SPIE) | _BV(SPE) | _BV(MSTR) | _BV(SPR1) | _BV(SPR0);
#else
	SPCR = _BV(SPE) | _BV(MSTR) | _BV(SPR1) | _BV(SPR0);
#endif
	// clear SPIF bit in SPSR
	IOReg = SPSR;
	IOReg = SPDR;
#endif

	/* Pin Change Mask Registers */
//	PCMSK0 = 0;		//_BV(PCINT0);
//	PCMSK1 = 0;
//	PCMSK2 = 0;		//_BV(PCINT23) | _BV(PCINT22) | _BV(PCINT21) | _BV(PCINT18);

	/* I/O-Ports */
	// PortB: Ziffernauswahl, Active Low!
	PORTB = 0xFF;
//	DDRB  = 0xFF;	nach Wartezeit

	// PortC: ADC, PC4: Relais1, PC5: Relais2 Active High
	PORTC = 0;
	DDRC = _BV(PC4) | _BV(PC5);

	// PortD: Segmentauswahl, Active High
	PORTD = 0;
//	DDRD = 0xFF;	nach Wartezeit

	/* Schlafmodus einstellen: Powerdown */
//	SMCR = _BV(SM1);
}

void dspl_uint8 (uint8_t pos, uint8_t dp, uint8_t value)
{
	// restliche Ziffern ermitteln
	if (pos != 0) {
		// zweiten vier Ziffern
		pos = 4;
	} else {
		// ersten 4 Ziffern
	}

	dspl.mem[pos + 3] = value % 10;	value /= 10;
	dspl.mem[pos + 2] = value % 10;	value /= 10;
	dspl.mem[pos + 1] = value % 10;
	dspl.mem[pos + 0] = DIGIT_0_;

	switch (dp)
	{
	case 1:		dspl.mem[pos + 2] |= SEGMENT_DP;	break;
	case 2:		dspl.mem[pos + 1] |= SEGMENT_DP;	break;
	case 3:		dspl.mem[pos + 0] |= SEGMENT_DP;	break;
	default:										break;
	}

	// Segmente konvertieren
	dspl_mem2seg (pos);
}

void dspl_hex_uint8 (uint8_t pos, uint8_t value)
{
	if (pos != 0) {
		// zweiten vier Ziffern
		pos = 4;
	} else {
		// ersten 4 Ziffern
	}

	dspl.mem[pos + 3] = value & 0xF;	value >>= 4;
	dspl.mem[pos + 2] = value & 0xF;
	dspl.mem[pos + 1] = DIGIT_BLANK;
	dspl.mem[pos + 0] = DIGIT_BLANK;

	// Segmente konvertieren
	dspl_mem2seg (pos);
}

void dspl_int8 (uint8_t pos, uint8_t dp, int8_t value)
{
	int16_t		tmp;

	tmp = value;

	switch (dp)
	{
	case 3: tmp *= 10;	/* no break */
	case 2: tmp *= 10;	/* no break */
	case 1: tmp *= 10;	/* no break */

	default:
		break;
	}

	// anzeigen
	dspl_int16 (pos, dp, tmp);
}

void dspl_int16 (uint8_t pos, uint8_t dp, int16_t value)
{
	uint8_t	first;

	if (value > 1999) {
		// Überlauf
		dspl_text (pos, TEXT_ID_OVF_PLUS);
		return;
	}

	if (value < -1999) {
		// Unterlauf
		dspl_text (pos, TEXT_ID_OVF_MINUS);
		return;
	}

	if (value < 0) {
		// Minus
		first = 1;
		value *= -1;
	} else {
		// Plus
		first = 0;
	}

	// erste Ziffer 0 oder 1
	if (value >= 1000)
		first += DIGIT_P1;	// Minus/Plus 1
	else
		first += DIGIT_P0;	// Minus/Plus 0

	// restliche Ziffern ermitteln
	if (pos != 0) {
		// zweiten vier Ziffern
		pos = 4;
	} else {
		// ersten 4 Ziffern
	}

	dspl.mem[pos + 3] = value % 10;	value /= 10;
	dspl.mem[pos + 2] = value % 10;	value /= 10;
	dspl.mem[pos + 1] = value % 10;
	dspl.mem[pos + 0] = first;

	switch (dp)
	{
	case 1:		dspl.mem[pos + 2] |= SEGMENT_DP;	break;
	case 2:		dspl.mem[pos + 1] |= SEGMENT_DP;	break;
	case 3:		dspl.mem[pos + 0] |= SEGMENT_DP;	break;
	default:										break;
	}

	// Segmente konvertieren
	dspl_mem2seg (pos);
}

void dspl_hex_uint16 (uint8_t pos, uint16_t value)
{
	uint8_t	first;

	if (value > 0x1FFF)
		return;

	// erste Ziffer 0 oder 1, ohne Plus/Minus
	if (value >= 0x1000)
		first = DIGIT_1_;	// 1
	else
		first = DIGIT_0_;	// 0

	// restliche Ziffern ermitteln
	if (pos != 0) {
		// zweiten vier Ziffern
		pos = 4;
	} else {
		// ersten 4 Ziffern
	}

	dspl.mem[pos + 3] = value & 0xF;	value >>= 4;
	dspl.mem[pos + 2] = value & 0xF;	value >>= 4;
	dspl.mem[pos + 1] = value & 0xF;
	dspl.mem[pos + 0] = first;

	// Segmente konvertieren
	dspl_mem2seg (pos);
}

void dspl_text (uint8_t pos, uint8_t txtId)
{
	// Position
	if (pos != 0) {
		// zweiten vier Ziffern
		pos = 4;
	} else {
		// ersten 4 Ziffern
	}

	// 4 Byte kopieren
	memcpy_P (&dspl.mem[pos], &text_tab[txtId], 4);

	// Segmente konvertieren
	dspl_mem2seg (pos);
}

void dspl_mem2seg (uint8_t pos)
{
	uint8_t	i;
	uint8_t	value;

	if (pos != 0) {
		// zweiten vier Ziffern
		pos = 4;
	} else {
		// ersten 4 Ziffern
	}

	for (i = pos; i < (4 + pos); i++) {
		// Ziffer holen
		value = dspl.mem[i];

		// Dezimalpunkt löschen
		value &= ~SEGMENT_DP;

		// Darstellung holen
		value = pgm_read_byte(&digits[value]);
	//	value = digits[value];

		// Dezimalpunkt addieren
		value |= (dspl.mem[i] & SEGMENT_DP);

		// und speichern
		dspl.seg[i] = value;
	}
}

void menu_readKey (uint8_t key)
{
	if (key == 0xFF) {
		// keine Taste gedrückt

	} else if (key >= (uint8_t)(ADC_KEY_MENU_MIN >> 2)
			&& key <= (uint8_t)(ADC_KEY_MENU_MAX >> 2))
	{
		key = MENU_KEY_MENU;

	} else if (key >= (uint8_t)(ADC_KEY_UP_MIN >> 2)
			&& key <= (uint8_t)(ADC_KEY_UP_MAX >> 2))
	{
		key = MENU_KEY_UP;

	} else if (key >= (uint8_t)(ADC_KEY_DOWN_MIN >> 2)
			&& key <= (uint8_t)(ADC_KEY_DOWN_MAX >> 2))
	{
		key = MENU_KEY_DOWN;

	} else if (key >= (uint8_t)(ADC_KEY_OK_MIN >> 2)
			&& key <= (uint8_t)(ADC_KEY_OK_MAX >> 2))
	{
		key = MENU_KEY_OK;

	} else {
		// außerhalb der definierten Grenzen
		key = 0xFF;
	}

	// Taste prüfen
	if (menu_cfg.keyLast != key) {
		// neue Taste
		menu_cfg.keyLast = key;
		menu_cfg.keyCnt = 0;
		menu_cfg.key = 0xFF;

	} else if (key != 0xFF) {
		// gültige Taste erkannt
		if (menu_cfg.keyCnt < MENU_KEY_CNT_MIN) {
			// zählen
			menu_cfg.keyCnt++;

		} else if (menu_cfg.keyCnt == MENU_KEY_CNT_MIN) {
			// Taste übernehmen
			menu_cfg.key = key;
			// nur einmal zählen
			menu_cfg.keyCnt++;

			// Änderungsbit setzen
			menu_cfg.changed = 1;

		} else {
			// wurde bereits gemeldet, Rücksetzen erfolgt ggf. nach Bearbeitung
		}
	}
}

void menu_printMenu (void)
{
	uint8_t		menu;
	uint8_t		i, id;
	int16_t		chId;

	// Anzeige nur bei Änderungen aktualisieren
	if (menu_cfg.changed != 0) {
		// Bit zurücksetzen
		menu_cfg.changed = 0;

		// zwischenspeichern für die Änderungserkennung
		menu = menu_cfg.menu;

		// Konfiguration aus dem Flash kopieren
		memcpy_P (&menu_setup, &menu_setup_tab[menu], sizeof(menu_setup));

		// Konfigurationsparameter in der zweiten Zeile
		if (menu_setup.para < CFG_PARA_END) {
			// Text in der ersten Zeile
			dspl_text (0, menu_setup.text_id);

			// Blinkender Parameterwert in der zweiten Zeile
			if (menu_cfg.flash != 0)
				dspl_text (1, TEXT_ID_BLANK);
			else
				dspl_int8 (1, 1, temp_cfg.para[menu_setup.para]);

		} else {

			if (menu_setup.para == MENU_PARA_TEMP) {
				// aktuelle Temperaturwerte

				// wenn ungültig -> Blinken
				if (temp_hist.valid[0] == 0 && menu_cfg.flash != 0) {
					dspl_text  (0, TEXT_ID_BLANK);
				} else {
#if TEMP_VAL_MAX > INT8_MAX
					dspl_int16 (0, 1, temp_hist.value[0]);
#else
					dspl_int8  (0, 1, temp_hist.value[0]);
#endif
				}
				// wenn ungültig -> Blinken
				if (temp_hist.valid[1] == 0 && menu_cfg.flash != 0) {
					dspl_text  (1, TEXT_ID_BLANK);
				} else {
#if TEMP_VAL_MAX > INT8_MAX
					dspl_int16 (1, 1, temp_hist.value[1]);
#else
					dspl_int8  (1, 1, temp_hist.value[1]);
#endif
				}
			} else {
				// MinMax-Verlauf: Index ausrechnen
				id = temp_hist.index;
				// die gewünschte Anzahl Stunden zurückgehen
				for (i = 0; i < menu_cfg.minMaxId; i++) {
					// Überlauf abfangen
					if (id > 0)
						id--;
					else
						id = 23;
				}

				// Kanal und Index
				chId = menu_cfg.minMaxId;

				// Auswahl
				switch (menu_setup.para)
				{
				case MENU_PARA_MAX_CH1:
					// Maximalwerte Kanal 1
					chId += 100;
					dspl_int16 (0, 2, chId);
#if TEMP_VAL_MAX > INT8_MAX
					dspl_int16 (1, 1, temp_hist.max_array[0][id]);
#else
					dspl_int8 (1, 1, temp_hist.max_array[0][id]);
#endif
					break;

				case MENU_PARA_MAX_CH2:
					// Maximalwerte Kanal 2
					chId += 200;
					dspl_int16 (0, 2, chId);
#if TEMP_VAL_MAX > INT8_MAX
					dspl_int16 (1, 1, temp_hist.max_array[1][id]);
#else
					dspl_int8 (1, 1, temp_hist.max_array[1][id]);
#endif
					break;

				case MENU_PARA_MIN_CH1:
					// Minimalwerte Kanal 1
					chId += 100;
					chId *= -1;
					dspl_int16 (0, 2, chId);
#if TEMP_VAL_MAX > INT8_MAX
					dspl_int16 (1, 1, temp_hist.min_array[0][id]);
#else
					dspl_int8 (1, 1, temp_hist.min_array[0][id]);
#endif
					break;

				case MENU_PARA_MIN_CH2:
					// Minimalwerte Kanal 2
					chId += 200;
					chId *= -1;
					dspl_int16 (0, 2, chId);
#if TEMP_VAL_MAX > INT8_MAX
					dspl_int16 (1, 1, temp_hist.min_array[1][id]);
#else
					dspl_int8 (1, 1, temp_hist.min_array[1][id]);
#endif
					break;

#if 0
				case MENU_PARA_SECONDS:
					// aktuelle Sekunden anzeigen
					dspl_text (0, TEXT_ID_BLANK);
					dspl_uint8 (1, 0, temp_hist.seconds);
					break;

				case MENU_PARA_MINUTES:
					// aktuelle Minuten anzeigen
					dspl_text (0, TEXT_ID_BLANK);
					dspl_uint8 (1, 0, temp_hist.minutes);
					break;

				case MENU_PARA_HOURS:
					// aktuelle Stunden anzeigen
					dspl_text (0, TEXT_ID_BLANK);
					dspl_uint8 (1, 0, temp_hist.hours);
					break;
#endif

				default:
					break;
				}
			}
		}

		// Tastendruck behandeln
		switch (menu_cfg.key)
		{
		case MENU_KEY_MENU:
			if (menu_setup.para < CFG_PARA_END) {
				// ungespeicherten Wert wiederherstellen
				switch (menu_setup.para)
				{
				case CFG_PARA_CH1_ON:	temp_cfg.para[menu_setup.para] = temp_ee_cfg.ch1_on;	break;
				case CFG_PARA_CH1_OFF:	temp_cfg.para[menu_setup.para] = temp_ee_cfg.ch1_off;	break;
				case CFG_PARA_CH2_ON:	temp_cfg.para[menu_setup.para] = temp_ee_cfg.ch2_on;	break;
				case CFG_PARA_CH2_OFF:	temp_cfg.para[menu_setup.para] = temp_ee_cfg.ch2_off;	break;

				default:
					break;
				}
			}
			if (menu_setup.menu_key_menu < MENU_NO) {
				// Folgemenü
				menu = menu_setup.menu_key_menu;

			} else if (menu_setup.para >= MENU_PARA_MAX_CH1
					&& menu_setup.para <= MENU_PARA_MIN_CH2) {
				// Verlaufsindex inkrementieren: nach links = ältere Werte
				if (menu_cfg.minMaxId < temp_hist.hours)
			//	if (menu_cfg.minMaxId < 23)
					menu_cfg.minMaxId++;

				// Menü aktualisieren
				menu_cfg.changed = 1;
			}
			break;

		case MENU_KEY_UP:
			if (menu_setup.menu_key_up < MENU_NO) {
				// Folgemenü
				menu = menu_setup.menu_key_up;

			} else if (menu_setup.para < CFG_PARA_END) {
				// Parameter inkrementieren
				temp_cfg.para[menu_setup.para] =
					menu_incr (	temp_cfg.para[menu_setup.para],
								temp_cfg.para[menu_setup.para_cmp],
								menu_setup.para_max);

				// Menü aktualisieren
				menu_cfg.changed = 1;
			} else {
				// nichts tun
			}
			break;

		case MENU_KEY_DOWN:
			if (menu_setup.menu_key_down < MENU_NO) {
				// Folgemenü
				menu = menu_setup.menu_key_down;

			} else if (menu_setup.para < CFG_PARA_END) {
				// Parameter dekrementieren
				temp_cfg.para[menu_setup.para] =
					menu_decr (	temp_cfg.para[menu_setup.para],
								temp_cfg.para[menu_setup.para_cmp],
								menu_setup.para_min);

				// Menü aktualisieren
				menu_cfg.changed = 1;
			} else {
				// nichts tun
			}
			break;

		case MENU_KEY_OK:
			if (menu_setup.para < CFG_PARA_END && menu_setup.para_cmp != PARA_NO) {
				// im Einstellungsmenü -> Speichern
				menu_saveConfig ();
			}

			if (menu_setup.menu_key_ok < MENU_NO) {
				// Folgemenü
				menu = menu_setup.menu_key_ok;

			} else if (menu_setup.para >= MENU_PARA_MAX_CH1
					&& menu_setup.para <= MENU_PARA_MIN_CH2) {
				// Verlaufsindex dekrementieren: nach rechts = neuere Werte
				if (menu_cfg.minMaxId > 0)
					menu_cfg.minMaxId--;

				// Menü aktualisieren
				menu_cfg.changed = 1;
			}
			break;

		default:
			// nichts tun
			break;
		}

		// neues Menü übernehmen, Änderungsbit setzen
		if (menu_cfg.menu != menu) {
			menu_cfg.menu = menu;
			menu_cfg.changed = 1;
		}

		// Tastendruck immer erstmal zurücksetzen
		menu_cfg.key = 0xFF;
	}
}

int8_t menu_incr (int8_t val, int8_t cmp, int8_t max)
{
	if (val < max)
		val += 1;

	if (val == cmp) {
		if (val < max)
			val = cmp + 1;
		else
			val = max - 1;
	}

	return val;
}

int8_t menu_decr (int8_t val, int8_t cmp, int8_t min)
{
	if (val > min)
		val -= 1;

	if (val == cmp) {
		if (val > min)
			val = cmp - 1;
		else
			val = min + 1;
	}

	return val;
}

void menu_loadConfig (void)
{
#if 0
	uint8_t	i, j;
	uint8_t	cnt1, cnt2;

	// die ersten beiden Schreibzähler lesen
	cnt1 = eeprom_read_byte ((const void *)(TEMP_CFG_EE_OFFSET + 0 * sizeof(temp_ee_cfg)));
	cnt2 = eeprom_read_byte ((const void *)(TEMP_CFG_EE_OFFSET + 1 * sizeof(temp_ee_cfg)));
	// vergleichen
	if (cnt1 == cnt2) {
		// die ersten beiden sind identisch -> noch nie beschrieben -> Standardwerte
		temp_cfg.cfg_id = 0xFF;

	} else {

		// Suche startet am Anfang
		temp_cfg.cfg_id = 0xFF;

		// Diskontinuität suchen
		for (i = 0; i < TEMP_CFG_EE_COUNT; i++) {
			// ersten Schreibzähler lesen
			cnt1 = eeprom_read_byte ((const void *)(TEMP_CFG_EE_OFFSET + i * sizeof(temp_ee_cfg)));

			// Adresse inkrementieren, Überlauf abfangen
			j = i + 1;
			if (j >= TEMP_CFG_EE_COUNT)
				j = 0;

			// nächsten Schreibzähler lesen
			cnt2 = eeprom_read_byte ((const void *)(TEMP_CFG_EE_OFFSET + j * sizeof(temp_ee_cfg)));

			// prüfen
			if (cnt1 != (cnt2 + 1)) {
				// nicht fortlaufend -> Abbruch

				// auf i steht jetzt der zuletzt beschriebene Parameterblock -> der wird geladen
				temp_cfg.cfg_id = i;
				// raus
				break;
			}
		}
	}
#else
	// einfach immer auf die 0 speichern, die 100k Löschzyklen erreicht keiner...
	temp_cfg.cfg_id = 0;
#endif

	// am Ende den zuletzt beschriebenen Eintrag laden
	if (temp_cfg.cfg_id < TEMP_CFG_EE_COUNT) {
		// ganzen Block lesen
		eeprom_read_block (&temp_ee_cfg, (const void *)(TEMP_CFG_EE_OFFSET + temp_cfg.cfg_id * sizeof(temp_ee_cfg)), sizeof(temp_ee_cfg));

#if 0
		// für's Schreiben zum nächsten Eintrag weiterschieben, Überlauf abfangen
		if (++temp_cfg.cfg_id >= TEMP_CFG_EE_COUNT)
			temp_cfg.cfg_id = 0;
#endif
		// Parameter kopieren
		temp_cfg.para[CFG_PARA_CH1_ON]  = temp_ee_cfg.ch1_on;
		temp_cfg.para[CFG_PARA_CH1_OFF] = temp_ee_cfg.ch1_off;
		temp_cfg.para[CFG_PARA_CH2_ON]  = temp_ee_cfg.ch2_on;
		temp_cfg.para[CFG_PARA_CH2_OFF] = temp_ee_cfg.ch2_off;

#if 0
		// Parameter prüfen

		// Kanal 1
		if (temp_cfg.ch1_on >= TEMP_CFG_CH1_MAX) {
			temp_cfg.ch1_on  = TEMP_CFG_CH1_MAX;
			// ggf. OFF korrigieren
			if (temp_cfg.ch1_off >= temp_cfg.ch1_on)
				temp_cfg.ch1_off  = temp_cfg.ch1_on - 1;
		}

		if (temp_cfg.ch1_off <= TEMP_CFG_CH1_MIN) {
			temp_cfg.ch1_off  = TEMP_CFG_CH1_MIN;
			// ggf. ON korrigieren
			if (temp_cfg.ch1_on <= temp_cfg.ch1_off)
				temp_cfg.ch1_on  = temp_cfg.ch1_off + 1;
		} else {
			// ggf. OFF korrigieren
			if (temp_cfg.ch1_off >= temp_cfg.ch1_on)
				temp_cfg.ch1_off  = temp_cfg.ch1_on - 1;
		}

		// Kanal 2
		if (temp_cfg.ch2_on >= TEMP_CFG_CH2_MAX) {
			temp_cfg.ch2_on  = TEMP_CFG_CH2_MAX;
			// ggf. OFF korrigieren
			if (temp_cfg.ch2_off >= temp_cfg.ch2_on)
				temp_cfg.ch2_off  = temp_cfg.ch2_on - 1;
		}

		if (temp_cfg.ch2_off <= TEMP_CFG_CH2_MIN) {
			temp_cfg.ch2_off  = TEMP_CFG_CH2_MIN;
			// ggf. ON korrigieren
			if (temp_cfg.ch2_on <= temp_cfg.ch2_off)
				temp_cfg.ch2_on  = temp_cfg.ch2_off + 1;
		} else {
			// ggf. OFF korrigieren
			if (temp_cfg.ch2_off >= temp_cfg.ch2_on)
				temp_cfg.ch2_off  = temp_cfg.ch2_on - 1;
		}
#endif
	} else {
		// Standardwerte benutzen, auf Platz 0 speichern
		temp_cfg.cfg_id = 0;

		// Standardwerte eintragen
		temp_cfg.para[CFG_PARA_CH1_ON] = 5;
		temp_cfg.para[CFG_PARA_CH1_OFF] = 10;

		temp_cfg.para[CFG_PARA_CH2_ON] = 20;
		temp_cfg.para[CFG_PARA_CH2_OFF] = 10;

		// Daten kopieren
		temp_ee_cfg.ch1_on  = temp_cfg.para[CFG_PARA_CH1_ON];
		temp_ee_cfg.ch1_off = temp_cfg.para[CFG_PARA_CH1_OFF];
		temp_ee_cfg.ch2_on  = temp_cfg.para[CFG_PARA_CH2_ON];
		temp_ee_cfg.ch2_off = temp_cfg.para[CFG_PARA_CH2_OFF];

		temp_ee_cfg.counter = 0;
	}
}

void menu_saveConfig (void)
{
	// Daten kopieren
	temp_ee_cfg.ch1_on  = temp_cfg.para[CFG_PARA_CH1_ON];
	temp_ee_cfg.ch1_off = temp_cfg.para[CFG_PARA_CH1_OFF];
	temp_ee_cfg.ch2_on  = temp_cfg.para[CFG_PARA_CH2_ON];
	temp_ee_cfg.ch2_off = temp_cfg.para[CFG_PARA_CH2_OFF];

	// Zähler erhöhen
//	temp_ee_cfg.counter++;

	// ganzen Block schreiben
	eeprom_write_block (&temp_ee_cfg, (void *)(TEMP_CFG_EE_OFFSET + temp_cfg.cfg_id * sizeof(temp_ee_cfg)), sizeof(temp_ee_cfg));

#if 0
	// zum nächsten Eintrag weiterschieben, Überlauf abfangen
	if (++temp_cfg.cfg_id >= TEMP_CFG_EE_COUNT)
		temp_cfg.cfg_id = 0;
#endif
}

void temp_startTemp (void)
{
	if (oneWire_reset() != 0) {
		// Adresse überspringen
		oneWire_writeByte (ONE_WIRE_CMD_SKIP_ROM);

		// Kommando senden
		oneWire_writeByte (ONE_WIRE_CMD_CONVERT_T);
	}
}

void temp_readTemp (void)
{
	int16_t		temp;
	uint8_t		*data;
	uint8_t		i, i_max, n;
	uint8_t		byte;

	// maximal 2 Sensoren einlesen und speichern
	if (oneWire.dev_count < 2)
		i_max = oneWire.dev_count;
	else
		i_max = 2;

	for (i = 0; i < i_max; i++) {
		// Bit zurücksetzen
		temp_hist.valid[i] = 0;

		// Sensor addressieren
		if (oneWire_selectDev(i) != 0) {
			// CRC zurücksetzen
			oneWire.crc8 = 0;

			// Kommando: Speicher lesen
			oneWire_writeByte (ONE_WIRE_CMD_RD_SCRATCH);

			// uint8-Zeiger auf den Zielspeicher
			data = ((uint8_t *)&oneWire.data);

			// 9 Byte lesen
			for (n = 0; n < 9; n++, data++) {
				// Byte lesen
				byte = oneWire_readByte ();
				// speichern
				*data = byte;

				// CRC aktualisieren
				oneWire_updateCRC (byte);
			}

			// crc prüfen
			if (oneWire.crc8 == 0) {
				// Daten verarbeiten, 12Bit Auflösung ist Standard
				// 2 Byte zusammenführen
				temp = (oneWire.data.temp_hi << 8) | oneWire.data.temp_lo;

				// 1 Bit entspricht 0.0625 °C = 1 / 16 °C
#if TEMP_VAL_MAX > INT8_MAX
				// mal (0.0625 * 10) => mal 10 durch 16
				temp *= 10;
				temp >>= 4;	// /= 16;
#else
				// Das wirft die Nachkommastellen weg, und es reicht int8!
				temp /= 16;
#endif

				// aktuellen Wert speichern
				temp_hist.value[i] = temp;

				// Wert ist gültig
				temp_hist.valid[i] = 1;

			} else {
				// CRC-Fehler
			}
		} else {
			// Select fehlgeschlagen
		}
	}
}

uint8_t temp_incrSeconds (void)
{
	// Sekunden inkrementieren
	temp_hist.seconds++;

	// 625 / 3 = 208 1/3 -> alle 3 Intervalle eine Sekunde zusätzlich
	temp_hist.add_sec++;
	if (temp_hist.add_sec >= 2) {
		temp_hist.add_sec = 0;
		// Schaltsekunde
		temp_hist.seconds++;
	}

	// Zeitzähler
	if (temp_hist.seconds > 59) {
		// Minute ist voll
		temp_hist.seconds -= 60;
		temp_hist.minutes++;
	}

	if (temp_hist.minutes > 59) {
		// Stunde ist voll
		temp_hist.minutes = 0;

		// Stundenzahl ist automatisch die Anzahl der auf MinMax zu prüfenden Einträge
		// daher den 24ten Eintrag weglassen, der wird als nächstes überschrieben
		if (temp_hist.hours < 23)
			temp_hist.hours++;

		return 1;
	}

	return 0;
}

void temp_updCurMinMax (void)
{
	temp_val_t	tmp;
	uint8_t		i;
//	uint8_t		j;
	uint8_t		id;

	// aktueller Index
	id = temp_hist.index;

	for (i = 0; i < 2; i++) {
		// nur gültige Werte verwenden
		if (temp_hist.valid[i] != 0) {
			// zwischenspeichern
			tmp = temp_hist.value[i];

			// Spitzenwertarray aktualisieren
			if (temp_hist.min_array[i][id] > tmp)
				temp_hist.min_array[i][id] = tmp;

			if (temp_hist.max_array[i][id] < tmp)
				temp_hist.max_array[i][id] = tmp;

#if 0
			// Spitzenwerte der letzten 6, 12 und 24 Stunden
			for (j = 0; j < 3; j++) {
				if (temp_hist.min[i][j] > tmp)
					temp_hist.min[i][j] = tmp;

				if (temp_hist.max[i][j] < tmp)
					temp_hist.max[i][j] = tmp;
			}
#endif
		}
	}
}

void temp_updHistMinMax (void)
{
	uint8_t		id;
//	uint8_t		i, j;
//	temp_val_t	min, max;

#if 0
	// die beiden Sensoren einzeln auswerten
	for (i = 0; i < 2; i++) {

		// mit aktuellem Index beginnen
		id = temp_hist.index;

		min = TEMP_VAL_MAX;
		max = TEMP_VAL_MIN;

		// Spitzenwerte der letzten Stunden (max. 23)
		for (j = 0; j < temp_hist.hours; j++) {
			// Minimum
			if (min > temp_hist.min_array[i][id])
				min = temp_hist.min_array[i][id];
			// Maximum
			if (max < temp_hist.max_array[i][id])
				max = temp_hist.max_array[i][id];

			if (j < 6) {
				// Spitzenwerte der letzten 6 Stunden
				temp_hist.min[i][0] = min;
				temp_hist.max[i][0] = max;
			}
			if (j < 12) {
				// Spitzenwerte der letzten 12 Stunden
				temp_hist.min[i][1] = min;
				temp_hist.max[i][1] = max;
			}
			// Spitzenwerte der letzten 23 Stunden
			temp_hist.min[i][2] = min;
			temp_hist.max[i][2] = max;

			// rückwärts wandern
			if (id > 0)
				id--;
			else
				id = 23;
		}
	}
#endif

	// neuer Index
	id = temp_hist.index + 1;
	// Überlauf abfangen
	if (id > 23)
		id = 0;

	// Index merken
	temp_hist.index = id;

	// neue Arraywerte zurücksetzen
	temp_hist.min_array[0][id] = TEMP_VAL_MAX;
	temp_hist.min_array[1][id] = TEMP_VAL_MAX;

	temp_hist.max_array[0][id] = TEMP_VAL_MIN;
	temp_hist.max_array[1][id] = TEMP_VAL_MIN;
}




void temp_updOutput (void)
{
	temp_val_t	t_on, t_off, temp;
	uint8_t		i, output[2], highOn;

	for (i = 0; i < 2; i++) {

		if (temp_hist.valid[i] != 0) {

#if TEMP_VAL_MAX > INT8_MAX
			// Nachkommastelle hinzufügen
			t_on  = temp_cfg.para[CFG_PARA_CH1_ON  + i] * 10;
			t_off = temp_cfg.para[CFG_PARA_CH1_OFF + i] * 10;
#else
			t_on  = temp_cfg.para[CFG_PARA_CH1_ON  + i];
			t_off = temp_cfg.para[CFG_PARA_CH1_OFF + i];
#endif

			// das Ergebnis des Vergleichs braucht man mehrmals
			highOn = (t_on > t_off);

			// Quellenauswahl
			switch (OUTPUT_CHx_SRC(i))
			{
			case TEMP_SRC_0:
				// Kanal abhängig vom Wert der definierten Quelle
				temp = temp_hist.value[0];
				break;

			case TEMP_SRC_1:
				// Kanal abhängig vom Wert der definierten Quelle
				temp = temp_hist.value[1];
				break;

			default:
			case TEMP_SRC_0_OR_1:
				// Quellenauswahl abghängig von den Parametern
				if (highOn != 0) {
					// ON bei hohen Temperaturen, OFF bei niedrigeren
					// den größeren der beiden Werte
					if (temp_hist.value[0] > temp_hist.value[1])
						temp = temp_hist.value[0];
					else
						temp = temp_hist.value[1];
				} else {
					// ON bei niedrigen Temperaturen, OFF bei höheren
					// den kleineren der beiden Werte
					if (temp_hist.value[0] < temp_hist.value[1])
						temp = temp_hist.value[0];
					else
						temp = temp_hist.value[1];
				}
				break;

			case TEMP_SRC_DELTA:
				// Kanal abhängig von der absoluten Temperaturdifferenz
				temp = temp_hist.value[0] - temp_hist.value[1];
				if (temp < 0)
					temp *= -1;
				break;
			}

			// vergleichen
			if (highOn != 0) {
				// ON bei hohen Temperaturen, OFF bei niedrigeren
				if (temp >= t_on) {
					// CHx an
					output[i] = 1;

				} else if (temp <= t_off) {
					// CHx aus
					output[i] = 0;

				} else {
					// noch keine Änderung, alter Wert bleibt erhalten
					output[i] = output_data.current[i];
				}
			} else {
				// ON bei niedrigen Temperaturen, OFF bei höheren
				if (temp <= t_on) {
					// CHx an
					output[i] = 1;

				} else if (temp >= t_off) {
					// CHx aus
					output[i] = 0;

				} else {
					// noch keine Änderung, alter Wert bleibt erhalten
					output[i] = output_data.current[i];
				}
			}
		} else {
			// keine gültigen Daten vom Sensor -> ausschalten
			output[i] = 0;
		}
	}

	// Kanal 2 erhält als Eingangssignal die Veroderung aus Kanal 2 und T2 von Kanal 1
	output[1] |= output_data.reg2[0];

	// Verzögerungen berechnen
	for (i = 0; i < 2; i++) {
		// an oder aus -> vergleichen mit dem aktuellen Zustand
		if (output_data.current[i] != output[i]) {
			// neuen Zustand speichern
			output_data.current[i] = output[i];
			// Zähler zurücksetzen
			output_data.count[i] = 0;

		} else {
			// Zustand bleibt gleich
			if (output_data.count[i] < 0xFF)
				output_data.count[i] += 1;

			// Verzögerung 1 prüfen
			if (output_data.count[i] >= (TEMP_OUTPUT_1_COUNT - 1)) {
				// ins Ausgangregister übernehmen
				output_data.reg1[i] = output[i];
			}

			// Verzögerung 2 prüfen
			if (output_data.count[i] >= (TEMP_OUTPUT_2_COUNT - 1)) {
				// ins Ausgangregister übernehmen
				output_data.reg2[i] = output[i];
			}
		}
	}


	// einzeln ins Ausgangsregister schreiben
	// jeweils die um T1 verzögerten Zustände
	if (output_data.reg1[0] != 0)
		OUTPUT_CHx_REG |=  OUTPUT_CHx_BIT(0);
	else
		OUTPUT_CHx_REG &= ~OUTPUT_CHx_BIT(0);

	if (output_data.reg1[1] != 0)
		OUTPUT_CHx_REG |=  OUTPUT_CHx_BIT(1);
	else
		OUTPUT_CHx_REG &= ~OUTPUT_CHx_BIT(1);
}


#if ONE_WIRE_ENABLE
/*
 * Lese-/Schreib-Funktionen nach MAXIM Application Note 126:
 * "1-Wire Communication Through Software"
 *
 * Suchalgorithmus nach MAXIM Application Note 187:
 * "1-Wire Search Algorithm"
 *
 * Kommunikation mit DS18B20 laut Datenblatt
 */

uint8_t oneWire_reset (void)
{
	uint8_t		result;

	ONE_WIRE_DELAY_G;

	// drive DQ low
	ONE_WIRE_OUT_LO;
	ONE_WIRE_DELAY_H;

	// release the bus
	ONE_WIRE_RELEASE;
	ONE_WIRE_DELAY_I;

	// sample for presence pulse from slave
	result = ONE_WIRE_READ;

	// complete the reset sequence recovery
	ONE_WIRE_DELAY_J;

	// return 1 if presence pulse was 0 -> device present
	return (result == 0);
}

void oneWire_writeBit (uint8_t data)
{
	if (data != 0)
	{
		// write '1' bit
		// drive DQ low
		ONE_WIRE_OUT;
		ONE_WIRE_DELAY_A;

		// release the bus
		ONE_WIRE_RELEASE;

		// complete the time slot and 10us recovery
		ONE_WIRE_DELAY_B;
	}
	else
	{
		// write '0' bit
		// drive DQ low
		ONE_WIRE_OUT;
		ONE_WIRE_DELAY_C;

		// release the bus
		ONE_WIRE_RELEASE;
		ONE_WIRE_DELAY_D;
	}
}

void oneWire_writeByte (uint8_t data)
{
	// loop to write each bit in the byte, LSB first
	for (uint8_t i = 0; i < 8; i++) {
		// write the LSB
		oneWire_writeBit (data & 0x01);

		// shift the data byte for the next bit
		data >>= 1;
	}
}

uint8_t oneWire_readBit (void)
{
	uint8_t		result;

	// drive DQ low
	ONE_WIRE_OUT;
	ONE_WIRE_DELAY_A;

	// release the bus
	ONE_WIRE_RELEASE;
	ONE_WIRE_DELAY_E;

	// sample the bit value
	result = ONE_WIRE_READ;

	// complete the time slot and 10us recovery
	ONE_WIRE_DELAY_F;

	// return 1 if the sampled value was 1
	return (result != 0);
}

uint8_t oneWire_readByte (void)
{
	uint8_t		result;

	result = 0;

	for (uint8_t i = 0; i < 8; i++) {
		// shift the result to get it ready for the next bit
		result >>= 1;

		// if sampled bit value is 1, then set MSB
		if (oneWire_readBit())
			result |= 0x80;
	}

	return result;
}

uint8_t oneWire_findFirst (void)
{
	oneWire.dev_count = 0;
	oneWire.last_device = 0;
	oneWire.last_disc = 0;
//	oneWire.last_fam_disc = 0;

	return oneWire_search ();
}

uint8_t oneWire_findNext (void)
{
	return oneWire_search ();
}

uint8_t oneWire_search (void)
{
	uint8_t		id_bit_no;
	uint8_t		last_zero, rom_byte_no, result;
	uint8_t		id_bit, id_bit_cmp;
	uint8_t		rom_byte_mask, direction;

	// initialize for search
	id_bit_no = 1;
	last_zero = 0;
	rom_byte_no = 0;
	rom_byte_mask = 1;
	result = 0;
	oneWire.crc8 = 0;

	// exit if ROM buffer is full
	if (oneWire.dev_count >= ONE_WIRE_DEV_NO)
		return 0;

	// if the last call was not the last one
	if (oneWire.last_device == 0)
	{
		// 1-Wire reset
		if (oneWire_reset() == 0)
		{
			// reset the search
			oneWire.last_disc = 0;
			oneWire.last_device = 0;
		//	oneWire.last_fam_disc = 0;

			return 0;
		}

		// issue the search command
		oneWire_writeByte (ONE_WIRE_CMD_SRCH_ROM);

		// loop to do the search
		do
		{
			// read the bit and its complement
			id_bit = oneWire_readBit ();
			id_bit_cmp = oneWire_readBit ();

			// check for no device on 1-Wire
			if (id_bit == 1 && id_bit_cmp == 1)
			{
				// no device responded
				break;
			}
			else
			{
				// all devices coupled have 0 or 1
				if (id_bit != id_bit_cmp)
				{
					// bit write value for search
					direction = id_bit;
				}
				else
				{
					// if this discrepancy is before the last discrepancy
					// on a previous next then pick the same as last time
					if (id_bit_no < oneWire.last_disc)
					{
						direction = ((oneWire.rom[oneWire.dev_count][rom_byte_no] & rom_byte_mask) > 0);
					}
					else
					{
						// if equal to last pick 1, if not then pick 0
						direction = (id_bit_no == oneWire.last_disc);
					}

					// if 0 was picked the revord its position
					if (direction == 0)
					{
						last_zero = id_bit_no;

					//	// check for last discrepancy in family
					//	if (last_zero < 9)
					//		oneWire.last_fam_disc = last_zero;
					}
				}

				// set or clear the bit in the ROM byte rom_byte_no with mask rom_byte_mask
				if (direction != 0)
					oneWire.rom[oneWire.dev_count][rom_byte_no] |=  rom_byte_mask;
				else
					oneWire.rom[oneWire.dev_count][rom_byte_no] &= ~rom_byte_mask;

				// serial number search direction write bit
				oneWire_writeBit (direction);

				// increment the byte counter and shift the mask
				id_bit_no++;
				rom_byte_mask <<= 1;

				// if the mask is 0 the got to new ROM byte and reset mask
				if (rom_byte_mask == 0)
				{
					oneWire_updateCRC (oneWire.rom[oneWire.dev_count][rom_byte_no]);
					rom_byte_no++;
					rom_byte_mask = 1;
				}
			}
		}
		while (rom_byte_no < 8);	// loop through all ROM bytes 0-7

		// if the search was successful
		if (!((id_bit_no < 64) || (oneWire.crc8 != 0)))
		{
			// search successful
			oneWire.last_disc = last_zero;

			// check for last device
			if (oneWire.last_disc == 0)
				oneWire.last_device = 1;

			result = 1;
		}
	}

	// if no device found then reset counters so next 'search' will be like a first
	if (result == 0 || oneWire.rom[oneWire.dev_count][0] == 0)
	{
		oneWire.last_device = 0;
		oneWire.last_disc = 0;
	//	oneWire.last_fam_disc = 0;
		result = 0;
	}
	else
	{
		// next device
		oneWire.dev_count++;
	}

	return result;
}

uint8_t oneWire_selectDev (uint8_t dev)
{
	uint8_t	i;

	// Reset
	if (/*dev >= oneWire.dev_count ||*/ oneWire_reset() == 0) {
		// ungültiges Gerät oder kein Presence-Pulse
		return 0;
	}

	// Befehl senden
	oneWire_writeByte (ONE_WIRE_CMD_MATCH_ROM);

	// Adresse byteweise senden
	for (i = 0; i < 8; i++)
		oneWire_writeByte (oneWire.rom[dev][i]);

	// alles okay
	return 1;
}

void oneWire_updateCRC (uint8_t byte)
{
	for (uint8_t bit = 0; bit < 8; bit++) {

		// Bits vergleichen, LSB first
		if (((oneWire.crc8 & 0x80) != 0) != ((byte & 0x01) != 0))
			oneWire.crc8 = (oneWire.crc8 << 1) ^ CRC_1WIRE_POLY;
		else
			oneWire.crc8 <<= 1;

		// nächstes Bit
		byte >>= 1;
	}
}

#endif	// ONE_WIRE_ENABLE


ISR (TIMER0_COMPA_vect)
{
#if 0
	uint8_t	digit;

	digit = dspl.digit;

	// alles kurz aus, sonst überlappen sich die Ziffern
	PORTB = 0xFF;

	// fertigen Wert ausgeben
	PORTD = dspl.seg[digit];	// & dspl.mask;

	// Ziffer ein (Active Low)
	PORTB = ~(1 << digit);

	// Umlauf
	if (++digit >= DIGIT_NO)
		digit = 0;

	// speichern
	dspl.digit = digit;
#endif
}

ISR (TIMER2_COMPA_vect)
{
	uint8_t	digit;

	digit = dspl.digit;

	// alles kurz aus, sonst überlappen sich die Ziffern
	PORTB = 0xFF;

	// fertigen Wert ausgeben
	PORTD = dspl.seg[digit];

	// Ziffer ein (Active Low)
	PORTB = ~(1 << digit);

	// Umlauf
	if (++digit >= DIGIT_NO)
		digit = 0;

	// speichern
	dspl.digit = digit;
}

ISR (ADC_vect)
{
	uint8_t		src, resL, resH;

	// Ergebnis lesen
	resL = ADCL;
	resH = ADCH;

	// Quelle
	src = adc_data.source;

	// in Tabelle speichern
	adc_data.mem[src - ADMUX_MIN] = (resH << 8) | resL;

	// nächste Quelle
	if (++src > ADMUX_MAX) {
		src = ADMUX_MIN;

		// Ergebnisse verarbeiten
		adc_data.complete = 1;
	}
	// und auswählen
	adc_data.source = src;
	ADMUX = adc_data.source | ADMUX_REFSEL;
}

