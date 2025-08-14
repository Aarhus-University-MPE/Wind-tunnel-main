#include <Arduino.h>
#include "DFRobot_UI.h"
#include "DFRobot_GDL.h"
#include "DFRobot_Touch.h"
#include <ModbusRTU.h>

// Display and UI
#define TFT_DC 3
#define TFT_CS 10
#define TFT_RST 2
const int margin = 10;
float tempSetPoint = 0;
float setPoint = 0;
bool isRunning = false;

// Track last values and timing
static float lastTempSetPoint = -1000, lastSetPoint = -1000;
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
float targetSpeedPercent = 0;
bool keepSendingSpeed = false;

// Motor register
#define REG_SPEED 102

struct SetpointMap
{
  float setpoint; // in m/s
  float percent;  // 0-100 %
};

// Your mapping table: you can add more points here
const SetpointMap mapTable[] = {
    {0.0, 0.0},
    {3.0, 10.0},
    {6.0, 20.0},
    {9.0, 30.0},
    {12.0, 40.0},
    {15.0, 50.0},
    {18.0, 60.0},
    {21.0, 70.0},
    {24.0, 80.0},
    {27.0, 90.0},
    {30.0, 100.0},
};

float mapSetPointToPercent(float spd)
{
  int tableSize = sizeof(mapTable) / sizeof(mapTable[0]);

  // Clamp input to the bounds of the table
  if (spd <= mapTable[0].setpoint)
    return mapTable[0].percent;
  if (spd >= mapTable[tableSize - 1].setpoint)
    return mapTable[tableSize - 1].percent;

  // Find interval spd fits into and linearly interpolate
  for (int i = 0; i < tableSize - 1; i++)
  {
    if (spd >= mapTable[i].setpoint && spd < mapTable[i + 1].setpoint)
    {
      float spanSet = mapTable[i + 1].setpoint - mapTable[i].setpoint;
      float spanPct = mapTable[i + 1].percent - mapTable[i].percent;
      float frac = (spd - mapTable[i].setpoint) / spanSet;
      return mapTable[i].percent + frac * spanPct;
    }
  }

  // Should never get here
  return 0;
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
void sendSpeedToAll(float percent)
{
  if (percent < 0)
    percent = 0;
  if (percent > 100)
    percent = 100;
  uint16_t value = (uint16_t)(percent * 100); // 0–100% -> 0–10000

  for (uint8_t i = 0; i < sizeof(motorIDs); i++)
  {
    lastModbusID = motorIDs[i]; // Track current motor ID
    mb.writeHreg(motorIDs[i], REG_SPEED, value, cb);
    while (mb.slave())
      mb.task();
  }
  Serial.print("Sent speed ");
  Serial.print(percent);
  Serial.println("% to all motors");
}

// Button callback
void btnCallback(DFRobot_UI::sButton_t &btn, DFRobot_UI::sTextBox_t &)
{
  String text((char *)btn.text);
  if (text == "-")
  {
    if (tempSetPoint > 0)
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
  }
  else if (text == "START")
  {
    isRunning = true;
    Serial.println("System started");
  }
  else if (text == "STOP")
  {
    isRunning = false;
    sendSpeedToAll(0);
    Serial.println("System stopped");
  }
}

void setup()
{
  pinMode(33, OUTPUT);
  digitalWrite(33, LOW);
  Serial.begin(115200);
  Serial5.begin(9600);   // RS-485/MODBUS for fan motors
  Serial8.begin(115200); // RS-482/serial for anemometer
  mb.begin(&Serial5);
  mb.master();

  ui.begin();
  ui.setTheme(DFRobot_UI::MODERN);
  touch.setRotation(3);
  screen.setRotation(3);

  // Create UI Buttons
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
  mb.task(); // always run Modbus

  // Update targetSpeedPercent from setPoint (touchscreen) when running
  if (isRunning)
  {
    targetSpeedPercent = mapSetPointToPercent(setPoint);
  }

  // Serial input override (optional)
  if (Serial.available())
  {
    String input = Serial.readStringUntil('\n');
    input.trim();
    float spd = input.toFloat();
    targetSpeedPercent = spd;
  }

  // Keep sending speed every 500ms if active
  static uint32_t lastSend = 0;
  if (isRunning && millis() - lastSend > 500 && !mb.slave())
  {
    lastSend = millis();
    sendSpeedToAll(targetSpeedPercent);
  }

  if (Serial8.available()) // checks for serial data from the anemometer
  {
    String line = Serial8.readStringUntil('\n'); // read until newline
    line.trim();                                 // remove any extra spaces or \r
    if (line.length() > 0)
    {
      windSpeed = line.toFloat(); // convert text to float
    }
    Serial.print("wind speed recived: ");
    Serial.println(windSpeed);
  }

  if (lastTempSetPoint != tempSetPoint || lastSetPoint != setPoint)
  {
    lastTempSetPoint = tempSetPoint;
    lastSetPoint = setPoint;

    int displayWidth = screen.width() / 2 - margin;
    int clearHeight = margin * 9;
    int labelX = margin;

    // Clear SETPOINT section
    screen.fillRect(0, margin * 10, displayWidth, clearHeight, COLOR_RGB565_BLACK);

    // Draw label
    screen.setTextColor(COLOR_RGB565_WHITE);
    screen.setTextSize(2);
    screen.setCursor(labelX, margin * 10);
    screen.print("SETPOINT:");

    // Draw value
    char buffer[10];
    sprintf(buffer, "%.1f", tempSetPoint);
    uint16_t color = (tempSetPoint == setPoint) ? COLOR_RGB565_GREEN : COLOR_RGB565_RED;
    screen.setTextColor(color);
    screen.setTextSize(8);
    screen.setCursor(labelX, margin * 12);
    screen.print(buffer);
  }

  // Update WINDSPEED display at most once per 1000ms
  if ((millis() - lastWindUpdate >= 500) && lastWindSpeed != windSpeed)
  {
    lastWindUpdate = millis();
    lastWindSpeed = windSpeed;

    int displayWidth = screen.width() / 2 - margin;
    int clearHeight = margin * 9;
    int labelX = margin;

    // Clear WINDSPEED section
    screen.fillRect(0, 0, displayWidth, clearHeight, COLOR_RGB565_BLACK);

    // Draw label
    screen.setTextColor(COLOR_RGB565_WHITE);
    screen.setTextSize(2);
    screen.setCursor(labelX, margin * 1);
    screen.print("WINDSPEED:");

    // Draw value
    char buffer[10];
    sprintf(buffer, "%.1f", windSpeed);
    screen.setTextSize(8);
    screen.setCursor(labelX, margin * 3);
    screen.print(buffer);
  }
}
