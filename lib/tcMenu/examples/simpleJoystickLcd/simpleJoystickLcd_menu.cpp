/*
    The code in this file uses open source libraries provided by thecoderscorner

    DO NOT EDIT THIS FILE, IT WILL BE GENERATED EVERY TIME YOU USE THE UI DESIGNER
    INSTEAD EITHER PUT CODE IN YOUR SKETCH OR CREATE ANOTHER SOURCE FILE.

    All the variables you may need access to are marked extern in this file for easy
    use elsewhere.
 */

#include <tcMenu.h>
#include "simpleJoystickLcd_menu.h"

// Global variable declarations

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
LiquidCrystalRenderer renderer(lcd, 16, 2);
ArduinoAnalogDevice analogDevice;

// Global Menu Item declarations

const PROGMEM FloatMenuInfo minfoFlow3 = { "Flow 3", 7, 0xffff, 2, NO_CALLBACK };
FloatMenuItem menuFlow3(&minfoFlow3, NULL);
const PROGMEM FloatMenuInfo minfoFlow2 = { "Flow 2", 6, 0xffff, 2, NO_CALLBACK };
FloatMenuItem menuFlow2(&minfoFlow2, &menuFlow3);
const PROGMEM FloatMenuInfo minfoFlow1 = { "Flow 1", 5, 0xffff, 2, NO_CALLBACK };
FloatMenuItem menuFlow1(&minfoFlow1, &menuFlow2);
RENDERING_CALLBACK_NAME_INVOKE(fnFlowsRtCall, backSubItemRenderFn, "Flows", -1, NULL)
const PROGMEM SubMenuInfo minfoFlows = { "Flows", 4, 0xffff, 0, NO_CALLBACK };
BackMenuItem menuBackFlows(fnFlowsRtCall, &menuFlow1);
SubMenuItem menuFlows(&minfoFlows, &menuBackFlows, NULL);
const PROGMEM BooleanMenuInfo minfoOverrideAll = { "Override all", 8, 0xffff, 1, NO_CALLBACK, NAMING_YES_NO };
BooleanMenuItem menuOverrideAll(&minfoOverrideAll, false, &menuFlows);
const PROGMEM AnalogMenuInfo minfoTideGate3 = { "Tide Gate 3", 3, 6, 100, onTidalGate3, 0, 1, "%" };
AnalogMenuItem menuTideGate3(&minfoTideGate3, 0, &menuOverrideAll);
const PROGMEM AnalogMenuInfo minfoTideGate2 = { "Tide Gate 2", 2, 4, 100, onTidalGate2, 0, 1, "%" };
AnalogMenuItem menuTideGate2(&minfoTideGate2, 0, &menuTideGate3);
const PROGMEM AnalogMenuInfo minfoTideGate1 = { "Tide Gate 1", 1, 2, 100, onTidalGate1, 0, 1, "%" };
AnalogMenuItem menuTideGate1(&minfoTideGate1, 0, &menuTideGate2);
const PROGMEM ConnectorLocalInfo applicationInfo = { "Joystick", "2098c102-226d-4cd4-8847-f1f68b7974f4" };

// Set up code

void setupMenu() {
    lcd.begin(16, 2);
    lcd.configureBacklightPin(10);
    lcd.backlight();
    switches.initialise(ioUsingArduino(), true);
    switches.addSwitch(A3, NULL);
    switches.onRelease(A3, [](uint8_t /*key*/, bool held) {
            menuMgr.onMenuSelect(held);
        });
    setupAnalogJoystickEncoder(&analogDevice, A1, [](int val) {
            menuMgr.valueChanged(val);
        });
    menuMgr.initWithoutInput(&renderer, &menuTideGate1);
}

