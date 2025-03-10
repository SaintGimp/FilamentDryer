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
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RuntimeMenuItem.h>
#include "tcMenuAdaFruitGfx.h"

// all define statements needed
#define ENCODER_UP_PIN 5
#define ENCODER_DOWN_PIN 6
#define ENCODER_OK_PIN 9

// all variables that need exporting
extern AdaColorGfxMenuConfig gfxConfig;
extern Adafruit_SSD1306 gfx;
extern AdaFruitGfxMenuRenderer renderer;

// all menu item forward references.
extern AnalogMenuItem menuSettingsPETGTemp;
extern AnalogMenuItem menuSettingsPLATemp;
extern BackMenuItem menuBackSettings;
extern SubMenuItem menuSettings;
extern EnumMenuItem menuHeaterMode;
extern TextMenuItem menuHumidity;
extern TextMenuItem menuTemperature;
extern const ConnectorLocalInfo applicationInfo;

// Callback functions must always include CALLBACK_FUNCTION after the return type
#define CALLBACK_FUNCTION

void CALLBACK_FUNCTION menuChangeHeaterMode(int id);
void CALLBACK_FUNCTION menuChangePetgTemp(int id);
void CALLBACK_FUNCTION menuChangePlaTemp(int id);

void setupMenu();

#endif // MENU_GENERATED_CODE_H
