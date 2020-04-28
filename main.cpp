
#include "main.h"

//Modes


Dimmer lamp[4]={4,5,6,7};
PinButton myButton[4]={A0,A1,A2,A3};

	
// Arduino UNO has 5.0 volt with a max ADC value of 1023 steps
// ACS712 5A  uses 185 mV per A
// ACS712 20A uses 100 mV per A
// ACS712 30A uses  66 mV per A

ACS712  ACS(A7, 5.0, 1023, 185);
bool Chip_ready=0;
int16_t sensorValue=0;

//OPTIONS//

uint8_t mode=DIMMER_MODE;
bool slave=0;
uint8_t bright_min[4]={1,10,10,1};
bool bright_dim[4]={1,1,1,1};
int16_t temps[4]={0,0,0,0};
uint8_t adj_temp[4]={0,0,0,0};
uint8_t temp_num=0;
bool temp_spare=0;
bool aut[4]={0,0,0,0};



// the setup routine runs once when you press reset:
void setup() {
	cli();
	delay(1000);
	if(!slave) Serial.begin(115200);
	pinMode(3, OUTPUT);
	//digitalWrite(LED,LOW);
	sensorValue = analogRead(A7);
	delay(1000);
	sei();
	//Ios_setup();
	cli();
	//if(!slave) Serial.println("READY");
	pinMode(A5,OUTPUT);
	Onewire_setup();
	sei();
}

void ADC_loop(void)
{
	sensorValue = ACS.mA_AC();
	if (sensorValue<120) sensorValue=0;
	else sensorValue-=120;
}

uint32_t cnt=millis();

void loop() {
  Temp_loop();
  ADC_loop();
  Buttons_loop();
  if(!slave) Serial_read();
  
  //1 Second loop
  
  if((millis()-cnt)>1000)
  {
	  cnt=millis();
	  //if(temp_on) Read_Sensors();
	  
	  Onewire_loop();
	  //Collect_data();
	  if(mode==TEMPS_MODE) Temp_Control();
	  else if(mode==VALVE_MODE) Valve_timing();
	  //PORTD^=1<<3;	//Toggle LED
  }
  
}

ISR(TIMER2_COMPA_vect)
{
	One_wire_int();
}
