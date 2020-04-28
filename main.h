/*
 * main.h
 *
 * Created: 02.04.2020 2:12:31
 *  Author: markin
 */ 


#ifndef MAIN_H_
#define MAIN_H_


#include "ios.h"
#include "Serial.h"
#include "Arduino.h"
#include "Dimmer.h"
//#include "DS18B20.h"
#include "PinButton.h"
#include "ACS712.h"
#include "onewirehw.h"

#define LED		3
#define OUT0	4
#define OUT1	5
#define OUT2	6
#define OUT3	7

#define IN0	14
#define IN1	15
#define IN2	16
#define IN3	17

#define WIRE0	18
#define WIRE1	19

#define COMMON_MODE		0
#define VALVE_MODE		1
#define TEMPS_MODE		2
#define DIMMER_MODE		3

extern int16_t sensorValue;
extern bool Chip_ready, slave;
extern uint8_t mode, bright_min[4];
extern bool bright_dim[4];
extern Dimmer lamp[4];
extern PinButton myButton[4];
extern int16_t temps[4];
extern uint8_t adj_temp[4];
extern bool temp_spare;
extern bool aut[4];


#endif /* MAIN_H_ */