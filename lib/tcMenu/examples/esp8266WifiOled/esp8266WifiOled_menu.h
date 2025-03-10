/*
    The code in this file uses open source libraries provided by thecoderscorner

    DO NOT EDIT THIS FILE, IT WILL BE GENERATED EVERY TIME YOU USE THE UI DESIGNER
    INSTEAD EITHER PUT CODE IN YOUR SKETCH OR CREATE ANOTHER SOURCE FILE.

    All the variables you may need access to are marked extern in this file for easy
    use elsewhere.
 */

#ifndef MENU_GENERATED_CODE_H
#define MENU_GENERATED_CODE_H

#include <tcMenu.h>
#include <ESP8266WiFi.h>
#include "EthernetTransport.h"
#include <RemoteConnector.h>
#include <RuntimeMenuItem.h>
#include "tcMenuU8g2.h"

// all define statements needed
#define TCMENU_USING_PROGMEM true
#define ENCODER_PIN_A 0
#define ENCODER_PIN_B 1
#define ENCODER_PIN_OK 2

// all variables that need exporting
extern U8G2_SSD1306_128X64_NONAME_F_SW_I2C gfx;
extern U8g2MenuRenderer renderer;
extern IoAbstractionRef io8574;

// all menu item forward references.
extern IpAddressMenuItem menuIpAddress;
extern TextMenuItem menuPwd;
extern TextMenuItem menuSSID;
extern BackMenuItem menuBackConnectivity;
extern SubMenuItem menuConnectivity;
extern ActionMenuItem menuSaveAll;
extern EnumMenuItem menuWinOpening;
extern EnumMenuItem menuHeaterPower;
extern BackMenuItem menuBackSetup;
extern SubMenuItem menuSetup;
extern BooleanMenuItem menuElectricHeater;
extern BooleanMenuItem menuWindowOpen;
extern AnalogMenuItem menuCucumberTemp;
extern AnalogMenuItem menuTomatoTemp;
extern const ConnectorLocalInfo applicationInfo;

// Callback functions must always include CALLBACK_FUNCTION after the return type
#define CALLBACK_FUNCTION

void CALLBACK_FUNCTION onElectricHeater(int id);
void CALLBACK_FUNCTION onHeaterPower(int id);
void CALLBACK_FUNCTION onSaveAll(int id);
void CALLBACK_FUNCTION onWindowOpen(int id);
void CALLBACK_FUNCTION onWindowOpening(int id);

void setupMenu();

#endif // MENU_GENERATED_CODE_H
