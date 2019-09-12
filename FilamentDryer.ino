#include <Adafruit_SleepyDog.h>

// https://www.thecoderscorner.com/products/arduino-libraries/tc-menu/tcmenu-overview-quick-start/
#include "FilamentDryer_menu.h"

// http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
#include <PID_v1.h>

#include <Wire.h>
#include <Adafruit_Si7021.h>

// TODO: borrow parts of this library and modify it to read
// the temperature generated during the humidity reading, and
// also play with the bit resolution of the readings
Adafruit_Si7021 sensor = Adafruit_Si7021();

const byte serialBufferSize = 32;
char receivedSerialChars[serialBufferSize];
boolean newSerialData = false;

const byte triacPin = 12;
const byte zeroCrossPin = 11;

const byte zeroCrossCountLimit = 24;
volatile byte zeroCrossCounter = 0;
volatile byte zeroCrossMatch = 0;

int plaSetPoint = 50;
int petgSetPoint = 60;

double currentTemperature = 0;
double currentHumidity = 0;

double pidSetPoint = 0.0;
double pidInput = 0.0;
double pidOutput = 0.0; 

double kp = 1.7;  // 2
double ki = 0.025; // 0.023
double kd = 0.0;

// Increasing kp causes the PID loop to be more proactive about backing off
// before it hits the set point, but also increases the short-duration twitchy-ness
// of the output (which doesn't seem to affect the temp that much)

// P_ON_M specifies that Proportional on Measurement be used rather than the
// default P_ON_E (Proportional on Error)
PID heaterPid(&pidInput, &pidOutput, &pidSetPoint, kp, ki, kd, P_ON_M, DIRECT);

Adafruit_SSD1306 gfx(128, 32, &Wire);
AdaColorGfxMenuConfig gfxConfig;

void setup() {
  digitalWrite(triacPin, LOW);
  pinMode(triacPin, OUTPUT);
  pinMode(zeroCrossPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(zeroCrossPin), ZeroCross, RISING);

  // OLED buttons
  pinMode(9, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);

  Serial.begin(115200);
  //while (!Serial) ;
  
  int countdownMs = Watchdog.enable(4000);
  Serial.print("Enabled the watchdog with max countdown of ");
  Serial.print(countdownMs, DEC);
  Serial.println(" milliseconds.");
  
  if (!sensor.begin()) {
    Serial.println("Did not find Si7021 sensor!");
    while (true)
      ;
  }
  Serial.println("Found Si7021!");
  
  heaterPid.SetMode(AUTOMATIC);
  heaterPid.SetOutputLimits(0.0, zeroCrossCountLimit + 1);

  gfx.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  gfx.clearDisplay();
  gfx.display();
  
  PrepareOledDisplayConfig();
  setupMenu();
  UpdateDisplayValues();
  menuSettingsPLATemp.setCurrentValue(plaSetPoint, true);
  menuSettingsPETGTemp.setCurrentValue(petgSetPoint, true);
  menuHeaterMode.setCurrentValue(0, true);
  renderer.setResetCallback(OnMenuReset);
  renderer.setResetIntervalTimeSeconds(120);
  
  taskManager.scheduleFixedRate(1000, UpdateDisplayValues);
  taskManager.scheduleFixedRate(1000, DoSerialOutput);
}

void loop() {
  taskManager.runLoop();
  
  ReadSensors();
  ControlHeater();

  ReceiveSerialInput();
  ProcessSerialInput();
  
  Watchdog.reset();
}

void ReadSensors() {
  // Read humidity first, then read the temperature
  // that was obtained during the humidity measurement (TODO)
  currentHumidity = sensor.readHumidity();
  currentTemperature = sensor.readTemperature();
}

void UpdateDisplayValues() {
  char buffer[8];

  sprintf(buffer, "%d/%d C", (int)(currentTemperature + 0.5), (int)(pidSetPoint + 0.5)); 
  menuTemperature.setTextValue(buffer, true);

  sprintf(buffer, "%d%%", (int)(currentHumidity + 0.5)); 
  menuHumidity.setTextValue(buffer, true);
}

void OnMenuReset() {
  // BUG: this isn't called after power-on if the menu hasn't been touched
  if (abs(pidInput - pidSetPoint) < 1 || pidSetPoint == 0) { 
    renderer.takeOverDisplay(IdleMenu);
  }
}

void IdleMenu(unsigned int encoderValue, bool clicked) {
  if (clicked || (pidSetPoint != 0 && abs(pidInput - pidSetPoint) > 2)) {
    renderer.giveBackDisplay();
    return;
  }

  gfx.clearDisplay();
  gfx.display();  
}

void PrepareOledDisplayConfig() {
  makePadding(gfxConfig.titlePadding, 1, 1, 1, 1);
  makePadding(gfxConfig.itemPadding, 1, 1, 1, 1);
  makePadding(gfxConfig.widgetPadding, 2, 2, 0, 2);

  gfxConfig.bgTitleColor = WHITE;
  gfxConfig.fgTitleColor = BLACK;
  gfxConfig.titleFont = NULL;
  gfxConfig.titleBottomMargin = 1;
  gfxConfig.widgetColor = BLACK;
  gfxConfig.titleFontMagnification = 1;

  gfxConfig.bgItemColor = BLACK;
  gfxConfig.fgItemColor = WHITE;
  gfxConfig.bgSelectColor = BLACK;
  gfxConfig.fgSelectColor = WHITE;
  gfxConfig.itemFont = NULL;
  gfxConfig.itemFontMagnification = 1;

  gfxConfig.editIcon = loResEditingIcon;
  gfxConfig.activeIcon = loResActiveIcon;
  gfxConfig.editIconHeight = 6;
  gfxConfig.editIconWidth = 8;
}

void ControlHeater() {
  pidInput = currentTemperature;
  heaterPid.Compute();
  zeroCrossMatch = (byte)(pidOutput + 0.5);
}

void DoSerialOutput() {
  Serial.print(zeroCrossMatch * 4);
  Serial.print(" ");
  Serial.print(currentTemperature, 4);
  Serial.print(" ");
  Serial.print(pidSetPoint, 4);
  Serial.print(" ");
  Serial.println(currentHumidity, 4);  
}

void ReceiveSerialInput() {
  static byte bufferIndex = 0;
  char endMarker = '\n';
  char receivedCharacter;
 
  while (Serial.available() > 0 && newSerialData == false) {
    receivedCharacter = Serial.read();

    if (receivedCharacter != endMarker) {
      receivedSerialChars[bufferIndex] = receivedCharacter;
      bufferIndex++;
      if (bufferIndex >= serialBufferSize) {
        bufferIndex = serialBufferSize - 1;
      }
    }
    else {
      receivedSerialChars[bufferIndex] = '\0';
      bufferIndex = 0;
      newSerialData = true;
    }
  }  
}

void ProcessSerialInput() {
  if (!newSerialData) {
    return;
  }

  if (receivedSerialChars[0] == 'P') {
    kp = atof(receivedSerialChars + 2);
  }
  
  if (receivedSerialChars[0] == 'I') {
    ki = atof(receivedSerialChars + 2);
  }

  if (receivedSerialChars[0] == 'D') {
    kd = atof(receivedSerialChars + 2);
  }

  if (receivedSerialChars[0] == 'S') {
    pidSetPoint = atof(receivedSerialChars + 2);
  }

  heaterPid.SetTunings(kp, ki, kd);
  Serial.print("P = ");
  Serial.print(kp, 4);
  Serial.print(", I = ");
  Serial.print(ki, 4);
  Serial.print(", D = ");
  Serial.print(kd, 4);
  Serial.print(", S = ");
  Serial.println(pidSetPoint, 4);

  newSerialData = false;
}

void ZeroCross() {
  zeroCrossCounter++;
  if (zeroCrossCounter > zeroCrossCountLimit) {
    zeroCrossCounter = 0;
    digitalWrite(triacPin, HIGH);
  }

  // This needs to be >= because the match could move downward between interupts
  if (zeroCrossCounter >= zeroCrossMatch) {
    digitalWrite(triacPin, LOW);
  }
}

void CALLBACK_FUNCTION menuChangePlaTemp(int id) {
  plaSetPoint = menuSettingsPLATemp.getCurrentValue();
  SetPidSetPoint(menuHeaterMode.getCurrentValue());
}

void CALLBACK_FUNCTION menuChangePetgTemp(int id) {
  petgSetPoint = menuSettingsPETGTemp.getCurrentValue();
  SetPidSetPoint(menuHeaterMode.getCurrentValue());
}

void CALLBACK_FUNCTION menuChangeHeaterMode(int id) {
  SetPidSetPoint(menuHeaterMode.getCurrentValue());
}

void SetPidSetPoint(int filamentType) {
  switch (filamentType)
  {
    case 0:
      pidSetPoint = 0;
      break;
    case 1:
      pidSetPoint = plaSetPoint;
      break;
    case 2:
      pidSetPoint = petgSetPoint;
      break;
  }
}
