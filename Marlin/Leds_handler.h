/*
- Leds_manager.h - A class that manages the Led colours
Last Update: 31/10/2017
Author: Alejandro Garcia (S3mt0x)
*/
#ifndef _LEDS_MANAGER_h
#define _LEDS_HANDLER_h

#define LED_MODE0	0
#define LED_MODE1	1
#define LED_MODE2	2

#include "Configuration.h"
#include "Marlin.h"
#include "ConfigurationStore.h"
extern uint8_t led_mode_state;
extern void SetupTimer2();

#endif

