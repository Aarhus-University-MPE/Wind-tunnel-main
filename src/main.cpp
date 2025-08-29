#include <Arduino.h>
#include "DFRobot_UI.h"
#include "DFRobot_GDL.h"
#include "DFRobot_Touch.h"
#include <ModbusRTU.h>
#include "bitmap.h"
// sander was here
// =================== USER-ADJUSTABLE SETTINGS ===================
const uint32_t openLoopTimeMs = 5000; // Time to run open-loop before PID takes over
// PID parameters
float kp = 1.5;
float ki = 0.2;
float kd = 0.05;
// ================================================================

// Display and UI
#define TFT_DC 3
#define TFT_CS 10
#define TFT_RST 2
const int margin = 10;
float tempSetPoint = 5;
float setPoint = 5; //The actual saved setPoint. After "set" is pressed.
bool isRunning = false;
bool manualMode = false;
float manualSpeed = 0;

// Track last values and timing
static float lastTempSetPoint = -1000, lastSetPoint = -1000; //The temporary sey-point
static float lastWindSpeed = -1000;
static uint32_t lastWindUpdate = 0;

uint8_t lastModbusID = 0;

DFRobot_Touch_GT911 touch;
DFRobot_ILI9488_320x480_HW_SPI screen(TFT_DC, TFT_CS, TFT_RST);
DFRobot_UI ui(&screen, &touch);

// Modbus
ModbusRTU mb;
const uint8_t rs485DirectionPin = 34;
uint16_t windSpeedRaw = 0;
float windSpeed = 0.0;

// Motor IDs
const uint8_t motorIDs[] = {2, 3, 4};
float targetSpeed = 0;

// Motor register
#define REG_SPEED 102

// Mapping table
struct SetpointMap
{
  float setpoint; // in m/s
  float setting;  // 0-100 %
};

const SetpointMap mapTable[] = {
    {0.0, 0.0},
    {5.1, 15},
    {6.8, 100},
    {9.5, 350},
    {12.4, 750},
    {15.5, 1400},
    {19.9, 2500},
    {22.8, 3500},
    {26.1, 5000},
    {30.8, 7500},
    {34.6, 10000},
};

float mapSetPointToSetting(float spd) // function to map a setpoint (m/s) to a motor control value (0-10000)
{
  int tableSize = sizeof(mapTable) / sizeof(mapTable[0]);

  if (spd <= mapTable[0].setpoint)
    return mapTable[0].setting;
  if (spd >= mapTable[tableSize - 1].setpoint)
    return mapTable[tableSize - 1].setting;

  for (int i = 0; i < tableSize - 1; i++)
  {
    if (spd >= mapTable[i].setpoint && spd < mapTable[i + 1].setpoint)
    {
      float spanSet = mapTable[i + 1].setpoint - mapTable[i].setpoint;
      float spanPct = mapTable[i + 1].setting - mapTable[i].setting;
      float frac = (spd - mapTable[i].setpoint) / spanSet;
      return mapTable[i].setting + frac * spanPct;
    }
  }
  return 0;
}

// PID state
float pidIntegral = 0;
float pidLastError = 0;

bool pidActive = false;
uint32_t pidStartTime = 0;

void resetPidAndOpenLoop() // function to reset PID and enable open loop control.
{
  pidActive = false;
  pidStartTime = millis();
  pidIntegral = 0;
  pidLastError = 0;
  Serial.println("Open loop mode re-started");
}

// Modbus callback
bool cb(Modbus::ResultCode event, uint16_t, void *)
{
  if (event != Modbus::EX_SUCCESS)
  {
    Serial.print("Modbus error from ID ");
    Serial.print(lastModbusID);
    Serial.print(": 0x");
    Serial.println(event, HEX);
  }
  return true;
}

// Send speed to all motors
void sendSpeedToAll(float setting)
{

  for (uint8_t i = 0; i < sizeof(motorIDs); i++)
  {
    lastModbusID = motorIDs[i];
    mb.writeHreg(motorIDs[i], REG_SPEED, setting, cb);
    while (mb.slave())
      mb.task();
  }
  Serial.print("Sent speed ");
  Serial.print(setting);
  Serial.println("to all motors");
}

// Button callback
void btnCallback(DFRobot_UI::sButton_t &btn, DFRobot_UI::sTextBox_t &)
{
  String text((char *)btn.text);
  if (text == "-")
  {
    if (tempSetPoint > 5)
      tempSetPoint -= 0.5;
  }
  else if (text == "+")
  {
    if (tempSetPoint < 30)
      tempSetPoint += 0.5;
  }
  else if (text == "Set")
  {
    setPoint = tempSetPoint;
    if (isRunning)
      resetPidAndOpenLoop(); // restart open loop when changing setpoint
  }
  else if (text == "START")
  {
    isRunning = true;
    resetPidAndOpenLoop();
    Serial.println("System started");
  }
  else if (text == "STOP")
  {
    isRunning = false;
    pidActive = false;
    sendSpeedToAll(0);
    Serial.println("System stopped");
  }
}

// timer and flag to track connection to the anemometer.
elapsedMillis anemometerTimeOut;
bool anemometerLost = false;

void setup()
{
  pinMode(33, OUTPUT); // pin 33 is the direction pin for the anemometer RS485 bus. tied low to enable reciver.
  digitalWrite(33, LOW);
  Serial.begin(115200);  // debug
  Serial5.begin(9600);   // modbus motor
  Serial8.begin(115200); // RS485 anemometer

  mb.begin(&Serial5);
  mb.master();

  ui.begin();
  ui.setTheme(DFRobot_UI::MODERN);
  touch.setRotation(3);
  screen.setRotation(3);

  int buttonWidth = (screen.width() / 4) - (margin * 1.5);
  int buttonHeight = 50;
  int buttonY = screen.height() - buttonHeight - margin;

  DFRobot_UI::sButton_t &btn1 = ui.creatButton();
  btn1.setText((char *)"-");
  btn1.fontSize = 6;
  btn1.bgColor = COLOR_RGB565_RED;
  btn1.setCallback(btnCallback);
  ui.draw(&btn1, margin, buttonY, buttonWidth, buttonHeight);

  DFRobot_UI::sButton_t &btn2 = ui.creatButton();
  btn2.setText((char *)"+");
  btn2.fontSize = 6;
  btn2.bgColor = COLOR_RGB565_DGREEN;
  btn2.setCallback(btnCallback);
  ui.draw(&btn2, margin + buttonWidth + margin, buttonY, buttonWidth, buttonHeight);

  DFRobot_UI::sButton_t &btn3 = ui.creatButton();
  btn3.setText((char *)"Set");
  btn3.fontSize = 4;
  btn3.bgColor = COLOR_RGB565_DCYAN;
  btn3.setCallback(btnCallback);
  ui.draw(&btn3, margin, buttonY - buttonHeight - margin, buttonWidth * 2 + margin, buttonHeight);

  DFRobot_UI::sButton_t &btn4 = ui.creatButton();
  btn4.setText((char *)"START");
  btn4.fontSize = 4;
  btn4.bgColor = COLOR_RGB565_GREEN;
  btn4.setCallback(btnCallback);
  int btnRightX = screen.width() - (buttonWidth * 2 + margin);
  int btnRightY = buttonY - buttonHeight - margin;
  ui.draw(&btn4, btnRightX, btnRightY, buttonWidth * 2 + margin, buttonHeight);

  DFRobot_UI::sButton_t &btn5 = ui.creatButton();
  btn5.setText((char *)"STOP");
  btn5.fontSize = 4;
  btn5.bgColor = COLOR_RGB565_RED;
  btn5.setCallback(btnCallback);
  int btnStopY = buttonY;
  ui.draw(&btn5, btnRightX, btnStopY, buttonWidth * 2 + margin, buttonHeight);
}

void loop()
{
  ui.refresh();
  mb.task();

  // Check for serial manual speed input used for debug. if serial data (0-10000) has been recived the system goes into manual mode. need power cycle to exit.
  if (Serial.available())
  {
    String input = Serial.readStringUntil('\n');
    input.trim();
    float spd = input.toFloat();
    if (spd >= 0 && spd <= 10000)
    {
      manualMode = true; // Enter manual mode permanently
      manualSpeed = spd; // Store manual speed
      Serial.print("Manual mode activated, speed = ");
      Serial.println(manualSpeed);
    }
  }

  if (!manualMode) // ===== Normal PID/Open-loop operation =====
  {
    // Control logic
    if (isRunning)
    {
      if (!pidActive)
      {
        targetSpeed = mapSetPointToSetting(setPoint);
        if (millis() - pidStartTime >= openLoopTimeMs)
        {
          pidActive = true;
          Serial.println("PID engaged");
        }
      }
      else
      {
        float error = setPoint - windSpeed;
        pidIntegral += error;
        float derivative = error - pidLastError;
        pidLastError = error;

        float pidOutput = kp * error + ki * pidIntegral + kd * derivative;
        targetSpeed = constrain(mapSetPointToSetting(setPoint) + pidOutput, 0, 10000);
      }
    }
  }
  else // ===== Manual mode =====
  {
    if (isRunning)
    {
      targetSpeed = manualSpeed; // Fixed speed in manual mode
    }
    else
    {
      targetSpeed = 0; // Stop motors if not running
    }
  }

  // Send speed periodically if all conditions are met.
  static uint32_t lastSend = 0;
  if (isRunning && millis() - lastSend > 500 && !mb.slave() && !anemometerLost)
  {
    lastSend = millis();
    sendSpeedToAll(targetSpeed);
  }

  // Read anemometer
  if (Serial8.available())
  {
    if (anemometerLost == true) // resets warning if connection with anemometer is reestablished.
    {
      anemometerLost = false;
      screen.fillRect(230, 10, 250, 160, COLOR_RGB565_BLACK);
    }
    anemometerTimeOut = 0;
    String line = Serial8.readStringUntil('\n');
    line.trim();
    if (line.length() > 0)
      windSpeed = line.toFloat();
    // Serial.print("wind speed received: ");
    // Serial.println(windSpeed);
  }

  if (anemometerTimeOut >= 2500) // anemometer connection timeout to track connection and draw warning on screen.
  {
    if (anemometerLost == false)
    {
      anemometerLost = true;
      isRunning = false;
      pidActive = false;
      sendSpeedToAll(0);
      screen.drawRGBBitmap(300, 10, (const uint16_t *)warning, 100, 100);
      screen.setTextColor(COLOR_RGB565_WHITE);
      screen.setTextSize(2);
      screen.setCursor(230, 120);
      screen.print("Lost connection with");
      screen.setCursor(290, 145);
      screen.print("Anemometer");
    }
  }

  // updates the SETPOINT display
  if (lastTempSetPoint != tempSetPoint || lastSetPoint != setPoint)
  {
    lastTempSetPoint = tempSetPoint;
    lastSetPoint = setPoint;
    int displayWidth = screen.width() / 2 - margin;
    int clearHeight = margin * 9;
    int labelX = margin;
    screen.fillRect(0, margin * 10, displayWidth, clearHeight, COLOR_RGB565_BLACK);
    screen.setTextColor(COLOR_RGB565_WHITE);
    screen.setTextSize(2);
    screen.setCursor(labelX, margin * 10);
    screen.print("SETPOINT:");
    char buffer[10];
    sprintf(buffer, "%.1f", tempSetPoint);
    uint16_t color = (tempSetPoint == setPoint) ? COLOR_RGB565_GREEN : COLOR_RGB565_RED;
    screen.setTextColor(color);
    screen.setTextSize(8);
    screen.setCursor(labelX, margin * 12);
    screen.print(buffer);
  }

  // updates the WINDSPEED display
  if ((millis() - lastWindUpdate >= 500) && lastWindSpeed != windSpeed)
  {
    lastWindUpdate = millis();
    lastWindSpeed = windSpeed;
    int displayWidth = screen.width() / 2 - margin;
    int clearHeight = margin * 9;
    int labelX = margin;
    screen.fillRect(0, 0, displayWidth, clearHeight, COLOR_RGB565_BLACK);
    screen.setTextColor(COLOR_RGB565_WHITE);
    screen.setTextSize(2);
    screen.setCursor(labelX, margin * 1);
    screen.print("WINDSPEED:");
    char buffer[10];
    sprintf(buffer, "%.1f", windSpeed);
    screen.setTextSize(8);
    screen.setCursor(labelX, margin * 3);
    screen.print(buffer);
  }
}
