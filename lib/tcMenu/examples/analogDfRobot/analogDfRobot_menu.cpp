/*
    The code in this file uses open source libraries provided by thecoderscorner

    DO NOT EDIT THIS FILE, IT WILL BE GENERATED EVERY TIME YOU USE THE UI DESIGNER
    INSTEAD EITHER PUT CODE IN YOUR SKETCH OR CREATE ANOTHER SOURCE FILE.

    All the variables you may need access to are marked extern in this file for easy
    use elsewhere.
 */

#include <tcMenu.h>
#include "analogDfRobot_menu.h"

// Global variable declarations

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
LiquidCrystalRenderer renderer(lcd, 16, 2);

// Global Menu Item declarations

const PROGMEM BooleanMenuInfo minfoLED2 = { "LED 2", 4, 3, 1, onLed2, NAMING_ON_OFF };
BooleanMenuItem menuLED2(&minfoLED2, false, NULL);
const PROGMEM BooleanMenuInfo minfoLED1 = { "LED 1", 3, 2, 1, onLed1, NAMING_ON_OFF };
BooleanMenuItem menuLED1(&minfoLED1, false, &menuLED2);
RENDERING_CALLBACK_NAME_INVOKE(fnLEDStatesRtCall, backSubItemRenderFn, "LED States", -1, NULL)
const PROGMEM SubMenuInfo minfoLEDStates = { "LED States", 2, 0xffff, 0, NO_CALLBACK };
BackMenuItem menuBackLEDStates(fnLEDStatesRtCall, &menuLED1);
SubMenuItem menuLEDStates(&minfoLEDStates, &menuBackLEDStates, NULL);
const PROGMEM AnalogMenuInfo minfoValueA0 = { "Value A0", 1, 0xffff, 1024, NO_CALLBACK, 0, 1, "" };
AnalogMenuItem menuValueA0(&minfoValueA0, 0, &menuLEDStates);
const PROGMEM ConnectorLocalInfo applicationInfo = { "DfRobot", "2ba37227-a412-40b7-94e7-42caf9bb0ff4" };

// Set up code

void setupMenu() {
    lcd.begin(16, 2);
    lcd.configureBacklightPin(10);
    lcd.backlight();
    pinMode(A0, INPUT);
    switches.initialise(inputFromDfRobotShield(), false);
    menuMgr.initForUpDownOk(&renderer, &menuValueA0, DF_KEY_DOWN, DF_KEY_UP, DF_KEY_SELECT);
}

