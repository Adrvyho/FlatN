#ifndef ONEWIREHW_H_
#define ONEWIREHW_H_

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif
//40, 185, 105, 84, 10, 0, 0, 209

//40, 255, 127, 169, 208, 22, 5, 16
//#define ONEWIRE_USE_PORTB
//#define ONEWIRE_DEVICES 1

#define OW_MATCH_ROM    0x55
#define OW_SKIP_ROM     0xCC
#define OW_SEARCH_ROM   0xF0

#define OW_SEARCH_FIRST 0xFF        // start new search
#define OW_PRESENCE_ERR 0xFF
#define OW_DATA_ERR     0xFE
#define OW_LAST_DEVICE  0x00        // last device found

#define OW_RECOVERY_TIME         10 /* usec */

// rom-code size including CRC
#define OW_ROMCODE_SIZE 8
#define MAX_1W_BUFFER_BYTES 16
#define ONEWIRECYCLEMKS 70
#define ONEWIRECYCLESINMS (1000/ONEWIRECYCLEMKS)


// DS18x20 EERPROM support disabled(0) or enabled(1) :
#define DS18X20_EEPROMSUPPORT     1
// decicelsius functions disabled(0) or enabled(1):
#define DS18X20_DECICELSIUS       1
// max. resolution functions disabled(0) or enabled(1):
#define DS18X20_MAX_RESOLUTION    1
// extended output via UART disabled(0) or enabled(1) :
#define DS18X20_VERBOSE           1


/* return values */
#define DS18X20_OK                0x00
#define DS18X20_ERROR             0x01
#define DS18X20_START_FAIL        0x02
#define DS18X20_ERROR_CRC         0x03

#define DS18X20_INVALID_DECICELSIUS  2000

#define DS18X20_POWER_PARASITE    0x00
#define DS18X20_POWER_EXTERN      0x01

#define DS18X20_CONVERSION_DONE   0x00
#define DS18X20_CONVERTING        0x01

/* DS18X20 specific values (see datasheet) */
#define DS18S20_FAMILY_CODE       0x10
#define DS18B20_FAMILY_CODE       0x28
#define DS1822_FAMILY_CODE        0x22

#define DS18X20_CONVERT_T         0x44
#define DS18X20_READ              0xBE
#define DS18X20_WRITE             0x4E
#define DS18X20_EE_WRITE          0x48
#define DS18X20_EE_RECALL         0xB8
#define DS18X20_READ_POWER_SUPPLY 0xB4

#define DS18B20_CONF_REG          4
#define DS18B20_9_BIT             0
#define DS18B20_10_BIT            (1<<5)
#define DS18B20_11_BIT            (1<<6)
#define DS18B20_12_BIT            ((1<<6)|(1<<5))
#define DS18B20_RES_MASK          ((1<<6)|(1<<5))

// undefined bits in LSB if 18B20 != 12bit
#define DS18B20_9_BIT_UNDF        ((1<<0)|(1<<1)|(1<<2))
#define DS18B20_10_BIT_UNDF       ((1<<0)|(1<<1))
#define DS18B20_11_BIT_UNDF       ((1<<0))
#define DS18B20_12_BIT_UNDF       0

// conversion times in milliseconds
#define DS18B20_TCONV_12BIT       750
#define DS18B20_TCONV_11BIT       DS18B20_TCONV_12_BIT/2
#define DS18B20_TCONV_10BIT       DS18B20_TCONV_12_BIT/4
#define DS18B20_TCONV_9BIT        DS18B20_TCONV_12_BIT/8
#define DS18S20_TCONV             DS18B20_TCONV_12_BIT

// constant to convert the fraction bits to cel*(10^-4)
#define DS18X20_FRACCONV          625

// scratchpad size in bytes
#define DS18X20_SP_SIZE           9

// DS18X20 EEPROM-Support
#define DS18X20_WRITE_SCRATCHPAD  0x4E
#define DS18X20_COPY_SCRATCHPAD   0x48
#define DS18X20_RECALL_E2         0xB8
#define DS18X20_COPYSP_DELAY      10 /* ms */
#define DS18X20_TH_REG            2
#define DS18X20_TL_REG            3

#define DS18X20_DECIMAL_CHAR      '.'

#define MAXSENSORS 4

void Onewire_loop(void);
void Onewire_setup(void);

class OneWireHW;
class Troller;
#define TROLLERS 2

extern OneWireHW _ohw_d0;
extern OneWireHW _ohw_d1;
extern Troller troll[TROLLERS];

struct ONEWIRE_PORT
{
    uint8_t pinsToStrobeHigh;      // нужно ли понизить пин в начале кадра
    uint8_t pinsToStrobeLow;       // нужно ли держать в середине кадра пин высоким
    uint8_t pinsToStrobeHighAfter; // нужно ли повышать пин в конце кадра
};

extern unsigned long OHW_CYCLES;


class OneWireHW
{
public:
    static ONEWIRE_PORT ONEWIRE_PORTC;
  //  static OneWireHW* devices[ONEWIRE_DEVICES];

    uint8_t buff[MAX_1W_BUFFER_BYTES];
    uint8_t maskForInterrupt; // маска нашего пина
    ONEWIRE_PORT *port;
    bool presence;

private:
    bool volatile reading; // читаем ли мы
    bool volatile writing;
    bool volatile aquiring;
    uint8_t volatile resetting;

    uint8_t currentReadBit;
    uint8_t bitsToRead;
    uint8_t currentWriteBit;
    uint8_t bitsToWrite;
	
	

public:
	uint8_t *boof_id;
	bool boof[64];
    OneWireHW(ONEWIRE_PORT *_port, uint8_t pin);
    bool IsReady();
    void StartTransaction(uint8_t bytesToWrite, uint8_t bytesToRead);

    static void StartTimer();
    static void EndTimer();
	bool reset(void);
	void skip(void);
	bool search(uint8_t *newAddr, bool search_mode /* = true */);
    bool PrepareForNextSlot(uint8_t currentPort);
	
private:
	
	bool OW_GET_IN();
	void OW_OUT_LOW();
	void OW_OUT_HIGH();
	void OW_DIR_IN();
	void OW_DIR_OUT();
	
public:
	
	
	uint8_t ow_reset(void);
	uint8_t ow_rom_search( uint8_t diff, uint8_t *id );
	uint8_t ow_byte_wr(uint8_t b);
	uint8_t ow_bit_io(uint8_t b);
	
};

enum TrollerState
{
	TEMP0_READ = 1,
	TEMP1_READ = 2,
	TEMP2_READ = 3,
	TEMP3_READ = 4,
};

enum TrollerAddr
{
	TEMP0_ADDR = 1,
	TEMP1_ADDR = 2,
	TEMP2_ADDR = 3,
	TEMP3_ADDR = 4,
};

class Troller
{
    uint8_t a_temp1[8];
    uint8_t a_temp2[8];
    uint8_t a_butt[8];

    uint8_t temp_state=0;

    unsigned long to_convert;
    unsigned long to_init;

    OneWireHW *hw;

public:
    uint8_t data_state;
	uint8_t Slaves_IDs[MAXSENSORS][OW_ROMCODE_SIZE];
    int temp1;
    int temp2;
    uint8_t butt;
	uint8_t nSensors;
	
    void init(OneWireHW &_hw);
    Troller();
	void ns(unsigned long now);
	void Search_Sensors(void);
private:
    void TransGetTemp(uint8_t *addr);
    void StartConvert();
	void StartConvertAdress(uint8_t *addr);
	int16_t Convert_temp(uint8_t *buff);
};


#define CRC8INIT    0x00
#define CRC8POLY    0x18              //0X18 = X^8+X^5+X^4+X^0

uint8_t crc8( uint8_t *data, uint16_t number_of_bytes_in_data );
void One_wire_int(void);
void Temp_loop(void);
#endif
