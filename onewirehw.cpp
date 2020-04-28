#include "onewirehw.h"
#include "main.h"

ONEWIRE_PORT OneWireHW::ONEWIRE_PORTC = {0,0,0};

OneWireHW _ohw_d0(&OneWireHW::ONEWIRE_PORTC, 4);
OneWireHW _ohw_d1(&OneWireHW::ONEWIRE_PORTC, 5);





uint8_t TEMPS[MAXSENSORS][OW_ROMCODE_SIZE];// = { 0x28, 0xB0, 0x4C, 0x64, 0x02, 0x00, 0x00, 0x00 };
uint8_t SLAVES[2][8];// = { 0x28, 0x78, 0xA3, 0x17, 0x03, 0x00, 0x00, 0x00 };
unsigned long OHW_CYCLES = 0;

bool OneWireHW::OW_GET_IN(void)
{
	if(PINC&maskForInterrupt) return 1;
	else return 0;
}

void OneWireHW::OW_OUT_LOW(void)
{
	PORTC &= ~maskForInterrupt;
}

void OneWireHW::OW_OUT_HIGH(void)
{
	PORTC |= maskForInterrupt;
}

void OneWireHW::OW_DIR_IN(void)
{
	DDRC &= ~maskForInterrupt;
}

void OneWireHW::OW_DIR_OUT(void)
{
	DDRC |= maskForInterrupt;
}

uint8_t OneWireHW:: ow_bit_io( uint8_t b )

{
	b&=1;
		OW_DIR_OUT();    // drive bus low
		delayMicroseconds(2);    // T_INT > 1usec accoding to timing-diagramm
		if ( b ) OW_DIR_IN(); // to write "1" release bus, resistor pulls high
		// "Output data from the DS18B20 is valid for 15usec after the falling
		// edge that initiated the read time slot. Therefore, the master must
		// release the bus and then sample the bus state within 15ussec from
		// the start of the slot."
		delayMicroseconds(15-2);
		if( OW_GET_IN() == 0 ) b = 0;  // sample at end of read-timeslot
		delayMicroseconds(60-15-2);
		OW_DIR_IN();
		delayMicroseconds(OW_RECOVERY_TIME); // may be increased for longer wires
		return b;
}

uint8_t OneWireHW::ow_reset(void)
{
	uint8_t err;
	cli();
	OW_OUT_LOW();
	OW_DIR_OUT();            // pull OW-Pin low for 480us
	delayMicroseconds(480);
	// set Pin as input - wait for clients to pull low
	OW_DIR_IN(); // input
	delayMicroseconds(64);       // was 66
	err = OW_GET_IN();   // no presence detect
	// if err!=0: nobody pulled to low, still high
	// after a delay the clients should release the line
	// and input-pin gets back to high by pull-up-resistor
	delayMicroseconds(480 - 64);       // was 480-66
	if( OW_GET_IN() == 0 ) err = 1;    // short circuit, expected low but got high
	sei();
	return err;
}

uint8_t OneWireHW::ow_byte_wr( uint8_t b )
{
	uint8_t i = 8, j;
	
	do {
		j = ow_bit_io( b & 1 );
		b >>= 1;
		if( j ) {
			b |= 0x80;
		}
	} while( --i );
	
	return b;
}

uint8_t OneWireHW::ow_rom_search( uint8_t diff, uint8_t *id )
{
	uint8_t i, j, next_diff;
	uint8_t b;
	
	if(ow_reset())
	{
		
		 return OW_PRESENCE_ERR;         // error, no device found <--- early exit!
	}
	
	cli();
	ow_byte_wr( OW_SEARCH_ROM );        // ROM search command
	next_diff = OW_LAST_DEVICE;         // unchanged on last device
	i = OW_ROMCODE_SIZE * 8;            // 8 bytes
	
	do 
	{
		j = 8;                          // 8 bits
		do {
			b = ow_bit_io(1);         // read bit
			if(ow_bit_io(1)) 
			{      // read complement bit
				if( b ) 
				{               // 0b11
					sei();
					
					return OW_DATA_ERR; // data error <--- early exit!
				}
			}
			else 
			{
				if( !b ) 
				{              // 0b00 = 2 devices
					if( diff > i || ((*id & 1) && diff != i) ) 
					{
						b = 1;          // now 1
						next_diff = i;  // next pass 0
					}
				}
			}
			ow_bit_io(b);             // write bit
			*id >>= 1;
			if( b ) {
				*id |= 0x80;            // store bit
			}
			
			i--;
			
		} while(--j);
		
		id++;                           // next byte
		
	} while(i);
	sei();
	return next_diff;                   // to continue search
}

OneWireHW::OneWireHW(ONEWIRE_PORT *_port, uint8_t pin)
{
    port  = _port;
    reading = false;
    writing = false;
    resetting = 0;
    aquiring = false;
    currentReadBit = 0;
    currentWriteBit = 0;
    maskForInterrupt = (1 << pin);
    presence = 0;
}

bool OneWireHW::IsReady()
{
    return !(reading | writing | resetting);
}


void OneWireHW::StartTransaction(uint8_t bytesToWrite, uint8_t bytesToRead)
{
    resetting = 1;
    currentWriteBit = 0;
    bitsToWrite = bytesToWrite * 8;
    writing = bytesToWrite;
    currentReadBit = 0;
    bitsToRead = bytesToRead * 8;
    reading = bytesToRead;
}



void inline SetMyTimerMkSec(uint8_t mks)
{
	OCR2A  = mks<<1; // 70mks
}

bool OneWireHW::PrepareForNextSlot(uint8_t currentPort)
{
    if (resetting)
    {
        if (resetting < 9)
        {
            // тут "начало" ресета.
            port->pinsToStrobeLow |= maskForInterrupt;
            resetting++;
            return 0;
        }
        port->pinsToStrobeHighAfter |= maskForInterrupt;
        if (resetting == 9)
        {
            presence = !(currentPort & maskForInterrupt);


            if (!presence)
			{
				resetting = aquiring = writing = reading = false; // ибо незачем читать или писать, если никого нет
				//return 1;
				
			}
                
            resetting++;
            return 0;
        }

        if (resetting == 16)
        {
            resetting = 0;
            return 0;
        }
        resetting++;
        return 0;
    }
    port->pinsToStrobeHighAfter |= maskForInterrupt;
	
    if (writing)
    {
        port->pinsToStrobeLow |= maskForInterrupt;

        uint8_t b = buff[currentWriteBit >> 3] & (1 << (currentWriteBit & 0x07));
        if (b)
            port->pinsToStrobeHigh |= maskForInterrupt;

        currentWriteBit++;
        // объявим, что мы уже не пишем, если это был последний бит
        writing = currentWriteBit < bitsToWrite;
        return 0;
    }
    if (aquiring)
    {
        uint8_t readbit = currentPort & maskForInterrupt;
        if (readbit)
            buff[currentReadBit >> 3] |= 1 << (currentReadBit & 0x07);
        else
            buff[currentReadBit >> 3] &= ~(1 << (currentReadBit & 0x07));

        currentReadBit++;
        // объявим, что мы уже не читаем, если это был последний бит
        aquiring = reading = currentReadBit < bitsToRead;
    }
    if (reading)
    {
        port->pinsToStrobeLow |= maskForInterrupt;
        port->pinsToStrobeHigh |= maskForInterrupt;
        aquiring = true;
        return 0;
    }
}



enum TIMESLOT_STEP_ENUM
{
    STEP_STROBE    = 0,
    STEP_RESTROBE  = 1,
    STEP_AQUIRE    = 2,
    STEP_PAUSE     = 3,
};

volatile TIMESLOT_STEP_ENUM TIMESLOT_STEP = STEP_STROBE;



void OneWireHW::StartTimer()
{
    // настроим таймер
	ASSR|=(1<<AS2);
    TCCR2A = (1<<WGM21); // CTC mode
    TCCR2B = (1<<CS21);  // Тактировать с коэффициентом деления 8.
	//OCR2A  = 2;
    SetMyTimerMkSec(50);  // пускай немного поничегонеделает
    TIMESLOT_STEP = STEP_STROBE;

    TIMSK2 |= (1<<OCIE2A); //  Разрешить прерывание по совпадению Т2.
}

void OneWireHW::EndTimer()
{
    TIMSK2 &= ~(1<<OCIE2A); //  Разрешить прерывание по совпадению Т2.
}


void inline memcpydk(volatile void *src, volatile void *dst, uint8_t size)
{
	for (uint8_t i = 0; i < size; i++)
	((uint8_t*)dst)[i] = ((uint8_t*)src)[i];
}


void Troller::init(OneWireHW &_hw)
    {
      hw = &_hw;
      data_state = 0;
      temp_state = 0;
      to_convert = 0;
      to_init = 0;
    }

Troller::Troller()
    {
      data_state = 0;
      temp_state = 0;
      hw = 0;
    }

void Troller::TransGetTemp(uint8_t addr[])
    {
      hw->buff[0] = 0x55;
      memcpydk(addr, hw->buff+1, 8);
      hw->buff[9] = 0xbe;
      hw->StartTransaction(10,9);
    }
	
void Troller::StartConvert()
    {
      hw->buff[0] = OW_SKIP_ROM;
      hw->buff[1] = DS18X20_CONVERT_T;
      hw->StartTransaction(2, 0);
      to_convert = OHW_CYCLES + 1200*ONEWIRECYCLESINMS; 
    }
	
void Troller::StartConvertAdress(uint8_t *addr)
{
	hw->buff[0] = OW_MATCH_ROM;
	memcpydk(addr, hw->buff+1, 8);
	hw->buff[9] = DS18X20_CONVERT_T;
	hw->StartTransaction(10, 0);
	to_convert = OHW_CYCLES + 800*ONEWIRECYCLESINMS; 
}
	
int16_t Troller::Convert_temp( uint8_t *buff)
{
	
	uint16_t measure;
	uint8_t  negative;
	int16_t  decicelsius;
	uint16_t fract;
	measure = (buff[0] | (buff[1] << 8));
	// check for negative
	if ( measure & 0x8000 )  {
		negative = 1;       // mark negative
		measure ^= 0xffff;  // convert to positive => (twos complement)++
		measure++;
	}
	else
	{
		negative = 0;
	}
	switch(buff[DS18B20_CONF_REG] & DS18B20_RES_MASK ) {
		case DS18B20_9_BIT:
		measure &= ~(DS18B20_9_BIT_UNDF);
		break;
		case DS18B20_10_BIT:
		measure &= ~(DS18B20_10_BIT_UNDF);
		break;
		case DS18B20_11_BIT:
		measure &= ~(DS18B20_11_BIT_UNDF);
		break;
		default:
		// 12 bit - all bits valid
		break;
	}
	decicelsius = (measure >> 4);
	decicelsius *= 10;

	// decicelsius += ((measure & 0x000F) * 640 + 512) / 1024;
	// 625/1000 = 640/1024
	fract = ( measure & 0x000F ) * 640;
	if ( !negative ) fract += 512;
	
	fract /= 1024;
	decicelsius += fract;

	if ( negative ) decicelsius = -decicelsius;
	

	if ( /* decicelsius == 850 || */ decicelsius < -550 || decicelsius > 1250 )
	{
		return DS18X20_INVALID_DECICELSIUS;
	}
	else return decicelsius;
	
}

uint8_t Sl_0[8]={40, 185, 105, 84, 10, 0, 0, 209};
uint8_t Sl_1[8]={40, 255, 127, 169, 208, 22, 5, 16};

void Troller::ns(unsigned long now)
    {
		//delay(10);
		if (!hw || !hw->IsReady()) return;
		
		if (temp_state == 0) 
		{
			temp_state = 1;
			StartConvert();
			return;
		}  
		if ((temp_state == 1)&&(to_convert < now)) 
		{
			temp_state = 2;
			TransGetTemp(Slaves_IDs[0]);
			return;
		}
		if (temp_state==2) 
		{
			temp_state = 3;
			
			temps[0] = Convert_temp(hw->buff);
			TransGetTemp(Slaves_IDs[1]);
			return;
		}
		if (temp_state==3)
		{
			temp_state = 0;
			/*
			if ( crc8( &hw->buff[0], DS18X20_SP_SIZE ) )
			{
				temp_state = 0;
				return;		// Checksum error
			}
			*/
			temps[1] = Convert_temp(hw->buff);
			return;
		}
		

    }
	
void Troller::Search_Sensors(void)
{
	uint8_t i;
	uint8_t id[OW_ROMCODE_SIZE];
	uint8_t diff;
	hw->ow_reset();
	nSensors = 0;
	
	diff = OW_SEARCH_FIRST;
	while ( diff != OW_LAST_DEVICE && nSensors < MAXSENSORS ) 
	{
		uint8_t go=1;
		uint8_t ret=DS18X20_OK;
		do 
		{
			diff = hw->ow_rom_search( diff, &id[0] );
			if ( diff == OW_PRESENCE_ERR || diff == OW_DATA_ERR ||
			diff == OW_LAST_DEVICE ) 
			{
				go  = 0;
				ret = DS18X20_ERROR;
			} else 
			{
				if ( id[0] == DS18B20_FAMILY_CODE || id[0] == DS18S20_FAMILY_CODE ||
				id[0] == DS1822_FAMILY_CODE ) go = 0;
			}
		} while (go);
		
		for ( i=0; i < OW_ROMCODE_SIZE; i++ )
		{
			Slaves_IDs[nSensors][i] = id[i];
			//Serial.print(id[i]);
			//Serial.print(' ');
			
		}
		//Serial.println();
		nSensors++;
	}
	
}


Troller troll[TROLLERS];
	
void Onewire_setup(void)
{
	troll[0].init(_ohw_d0);
	troll[0].Search_Sensors();
	for(uint8_t j=0; j<troll[0].nSensors;j++)
	{
	for (uint8_t i=0; i < OW_ROMCODE_SIZE; i++)
	{
		
		Serial.print(troll[0].Slaves_IDs[j][i]);
		Serial.print(' ');
		
	}
	Serial.println();	
	}
	
	OneWireHW::StartTimer();
	sei();
	
}

void Temp_loop(void)
{
	unsigned long now = OHW_CYCLES;
	troll[0].ns(now);
}

void Onewire_loop(void)
{
	//temps[0]=troll[0].temp1;
	Serial.print("temp 0=");
	Serial.println(temps[0]);
	Serial.print("temp 1=");
	Serial.println(temps[1]);
}

static ONEWIRE_PORT CURRENT_PORTC = {0,0,0};

void One_wire_int(void)
{
	
	
	if (TIMESLOT_STEP == STEP_STROBE)
	{
		
		SetMyTimerMkSec(8);
		TIMESLOT_STEP = STEP_RESTROBE;
		uint8_t pgl = CURRENT_PORTC.pinsToStrobeLow; // i hope it will use a register
		PORTC  &= ~pgl;  // теперь опустим эти пины
		DDRC   |=  pgl;  // пины, которые "дергаем", переключаем как OUTPUT
		return;
	}

	if (TIMESLOT_STEP == STEP_RESTROBE)
	{
		SetMyTimerMkSec(7);
		TIMESLOT_STEP = STEP_AQUIRE;
		uint8_t pgh = CURRENT_PORTC.pinsToStrobeHigh; // i hope it will use a register
		PORTC  |=  pgh;  // теперь поднимем эти пины
		DDRC   &= ~pgh;  // пины, которые "дергаем", переключаем как INPUT
		return;
	}

	if (TIMESLOT_STEP == STEP_AQUIRE)
	{
		//PORTC^=(1<<5);
		SetMyTimerMkSec(38);
		TIMESLOT_STEP = STEP_PAUSE;
		uint8_t readedbitsc = PINC;

		// тут пускай нам все скажут, что ж мы будем делать в след тайм-слоте
		OneWireHW::ONEWIRE_PORTC.pinsToStrobeHigh = 0;
		OneWireHW::ONEWIRE_PORTC.pinsToStrobeHighAfter = 0;
		OneWireHW::ONEWIRE_PORTC.pinsToStrobeLow = 0;

		_ohw_d0.PrepareForNextSlot(readedbitsc);
		return;
	}
	if (TIMESLOT_STEP == STEP_PAUSE)
	{
		OHW_CYCLES++;
		SetMyTimerMkSec(7);
		TIMESLOT_STEP = STEP_STROBE;
		uint8_t pgh = CURRENT_PORTC.pinsToStrobeHighAfter; // тут по идее все, кто стробился
		PORTC |=  pgh;
		DDRC  |=  pgh;
		CURRENT_PORTC = OneWireHW::ONEWIRE_PORTC;
		return;
	}
	
}


uint8_t crc8( uint8_t *data, uint16_t number_of_bytes_in_data )
{
	uint8_t  crc;
	uint16_t loop_count;
	uint8_t  bit_counter;
	uint8_t  b;
	uint8_t  feedback_bit;
	
	crc = CRC8INIT;

	for (loop_count = 0; loop_count != number_of_bytes_in_data; loop_count++)
	{
		b = data[loop_count];
		
		bit_counter = 8;
		do {
			feedback_bit = (crc ^ b) & 0x01;
			
			if ( feedback_bit == 0x01 ) {
				crc = crc ^ CRC8POLY;
			}
			crc = (crc >> 1) & 0x7F;
			if ( feedback_bit == 0x01 ) {
				crc = crc | 0x80;
			}
			
			b = b >> 1;
			bit_counter--;
			
		} while (bit_counter > 0);
	}
	
	return crc;
}


