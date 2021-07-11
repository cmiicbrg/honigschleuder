#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library.
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET 4        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define PIN_L 8
#define PIN_R 7
#define PIN_START 2
#define PIN_SPEED 3
#define PIN_LED_R1 13
#define PIN_LED_R2 12
#define PIN_LED_R3 11
#define PIN_LED_R4 10
#define PIN_LED_R5 9
#define PIN_LED_L1 0
#define PIN_LED_L2 1
#define PIN_LED_L3 4
#define PIN_LED_L4 5
#define PIN_LED_L5 6

float maxMinutes = 10;
float minMinutes = 0.5;
float potiMax = 1023;
int leftPoti;
int rightPoti;
int speedPoti;
bool running = false;
#define AUTO_MODE 0
#define MANUAL_LEFT 1
#define MANUAL_RIGHT 2
int opMode = AUTO_MODE;

unsigned long runStartTime;
unsigned long setSpeedTimer = millis();
unsigned long speedTimerIntervall = 200;

#define LEFT 1
#define RIGHT 2
int currentDirection = 0;
#define firstRunSpeedPercent 15
#define secondRunSpeedPercent 20
#define thirdRunSpeedPercent 25
int currentRun = 0;
int currentSpeed = 0;
int targetSpeed = 0;

#define triggered true // controls interrupt handler
#define switched true  // value if the button switch has been pressed
#define debounce 10    // time to wait in milli secs
volatile bool interrupt_process_status = {
    !triggered // start with no switch press pending, ie false (!triggered)
};
bool initialisation_complete = false; // inhibit any interrupts until initialisation is complete

void button_interrupt_handler()
{
  if (initialisation_complete == true)
  {
    if (interrupt_process_status == !triggered)
    {
      if (digitalRead(PIN_START) == LOW)
      {
        interrupt_process_status = triggered;
      }
    }
  }
}

bool read_button()
{
  int button_reading;
  static bool switching_pending = false;
  static long int elapse_timer;
  if (interrupt_process_status == triggered)
  {
    button_reading = digitalRead(PIN_START);
    if (button_reading == LOW)
    {
      switching_pending = true;
      elapse_timer = millis();
    }
    if (switching_pending && button_reading == HIGH)
    {
      if (millis() - elapse_timer >= debounce)
      {
        switching_pending = false;
        interrupt_process_status = !triggered;
        return switched;
      }
    }
  }
  return !switched;
}

float calcTimeMinutes(int poti, float fractionalPrecision = 2)
{
  float k = (maxMinutes + 0.499 - minMinutes) / (potiMax);
  float minutes = minMinutes + k * poti;
  return floor(minutes * fractionalPrecision) / fractionalPrecision;
}
int calcSpeedPercentage(int poti)
{
  float k = 100 / potiMax;
  float per = k * poti;
  return floor(per);
}

void showTimeLEDs(int leftTime, int rightTime)
{
  if (leftTime >= 10)
  {
    digitalWrite(PIN_LED_L1, 0);
    digitalWrite(PIN_LED_L2, 0);
    digitalWrite(PIN_LED_L3, 0);
    digitalWrite(PIN_LED_L4, 0);
    digitalWrite(PIN_LED_L5, 1);
  }
  else if (leftTime >= 9)
  {
    digitalWrite(PIN_LED_L1, 0);
    digitalWrite(PIN_LED_L2, 0);
    digitalWrite(PIN_LED_L3, 0);
    digitalWrite(PIN_LED_L4, 1);
    digitalWrite(PIN_LED_L5, 1);
  }
  else if (leftTime >= 8)
  {
    digitalWrite(PIN_LED_L1, 0);
    digitalWrite(PIN_LED_L2, 0);
    digitalWrite(PIN_LED_L3, 0);
    digitalWrite(PIN_LED_L4, 1);
    digitalWrite(PIN_LED_L5, 0);
  }
  else if (leftTime >= 7)
  {
    digitalWrite(PIN_LED_L1, 0);
    digitalWrite(PIN_LED_L2, 0);
    digitalWrite(PIN_LED_L3, 1);
    digitalWrite(PIN_LED_L4, 1);
    digitalWrite(PIN_LED_L5, 0);
  }
  else if (leftTime >= 6)
  {
    digitalWrite(PIN_LED_L1, 0);
    digitalWrite(PIN_LED_L2, 0);
    digitalWrite(PIN_LED_L3, 1);
    digitalWrite(PIN_LED_L4, 0);
    digitalWrite(PIN_LED_L5, 0);
  }
  else if (leftTime >= 5)
  {
    digitalWrite(PIN_LED_L1, 0);
    digitalWrite(PIN_LED_L2, 1);
    digitalWrite(PIN_LED_L3, 1);
    digitalWrite(PIN_LED_L4, 0);
    digitalWrite(PIN_LED_L5, 0);
  }
  else if (leftTime >= 4)
  {
    digitalWrite(PIN_LED_L1, 0);
    digitalWrite(PIN_LED_L2, 1);
    digitalWrite(PIN_LED_L3, 0);
    digitalWrite(PIN_LED_L4, 0);
    digitalWrite(PIN_LED_L5, 0);
  }
  else if (leftTime >= 3)
  {
    digitalWrite(PIN_LED_L1, 1);
    digitalWrite(PIN_LED_L2, 1);
    digitalWrite(PIN_LED_L3, 0);
    digitalWrite(PIN_LED_L4, 0);
    digitalWrite(PIN_LED_L5, 0);
  }
  else if (leftTime >= 2)
  {
    digitalWrite(PIN_LED_L1, 1);
    digitalWrite(PIN_LED_L2, 0);
    digitalWrite(PIN_LED_L3, 0);
    digitalWrite(PIN_LED_L4, 0);
    digitalWrite(PIN_LED_L5, 0);
  }
  else if (leftTime >= 0)
  {
    digitalWrite(PIN_LED_L1, 0);
    digitalWrite(PIN_LED_L2, 0);
    digitalWrite(PIN_LED_L3, 0);
    digitalWrite(PIN_LED_L4, 0);
    digitalWrite(PIN_LED_L5, 0);
  }
  if (rightTime >= 10)
  {
    digitalWrite(PIN_LED_R1, 0);
    digitalWrite(PIN_LED_R2, 0);
    digitalWrite(PIN_LED_R3, 0);
    digitalWrite(PIN_LED_R4, 0);
    digitalWrite(PIN_LED_R5, 1);
  }
  else if (rightTime >= 9)
  {
    digitalWrite(PIN_LED_R1, 0);
    digitalWrite(PIN_LED_R2, 0);
    digitalWrite(PIN_LED_R3, 0);
    digitalWrite(PIN_LED_R4, 1);
    digitalWrite(PIN_LED_R5, 1);
  }
  else if (rightTime >= 8)
  {
    digitalWrite(PIN_LED_R1, 0);
    digitalWrite(PIN_LED_R2, 0);
    digitalWrite(PIN_LED_R3, 0);
    digitalWrite(PIN_LED_R4, 1);
    digitalWrite(PIN_LED_R5, 0);
  }
  else if (rightTime >= 7)
  {
    digitalWrite(PIN_LED_R1, 0);
    digitalWrite(PIN_LED_R2, 0);
    digitalWrite(PIN_LED_R3, 1);
    digitalWrite(PIN_LED_R4, 1);
    digitalWrite(PIN_LED_R5, 0);
  }
  else if (rightTime >= 6)
  {
    digitalWrite(PIN_LED_R1, 0);
    digitalWrite(PIN_LED_R2, 0);
    digitalWrite(PIN_LED_R3, 1);
    digitalWrite(PIN_LED_R4, 0);
    digitalWrite(PIN_LED_R5, 0);
  }
  else if (rightTime >= 5)
  {
    digitalWrite(PIN_LED_R1, 0);
    digitalWrite(PIN_LED_R2, 1);
    digitalWrite(PIN_LED_R3, 1);
    digitalWrite(PIN_LED_R4, 0);
    digitalWrite(PIN_LED_R5, 0);
  }
  else if (rightTime >= 4)
  {
    digitalWrite(PIN_LED_R1, 0);
    digitalWrite(PIN_LED_R2, 1);
    digitalWrite(PIN_LED_R3, 0);
    digitalWrite(PIN_LED_R4, 0);
    digitalWrite(PIN_LED_R5, 0);
  }
  else if (rightTime >= 3)
  {
    digitalWrite(PIN_LED_R1, 1);
    digitalWrite(PIN_LED_R2, 1);
    digitalWrite(PIN_LED_R3, 0);
    digitalWrite(PIN_LED_R4, 0);
    digitalWrite(PIN_LED_R5, 0);
  }
  else if (rightTime >= 2)
  {
    digitalWrite(PIN_LED_R1, 1);
    digitalWrite(PIN_LED_R2, 0);
    digitalWrite(PIN_LED_R3, 0);
    digitalWrite(PIN_LED_R4, 0);
    digitalWrite(PIN_LED_R5, 0);
  }
  else if (rightTime >= 0)
  {
    digitalWrite(PIN_LED_R1, 0);
    digitalWrite(PIN_LED_R2, 0);
    digitalWrite(PIN_LED_R3, 0);
    digitalWrite(PIN_LED_R4, 0);
    digitalWrite(PIN_LED_R5, 0);
  }
}

void showInfo()
{
  float leftTime = 0;
  float rightTime = 0;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);

  if (opMode == MANUAL_LEFT || opMode == MANUAL_RIGHT)
  {
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("Manuell");

    display.setTextSize(3);
    display.setCursor(0, 16);
    if (opMode == MANUAL_LEFT)
    {
      display.print("L ");
    }
    else if (opMode == MANUAL_RIGHT)
    {
      display.print("R ");
    }
    display.print(calcSpeedPercentage(1023 - analogRead(A0)));
    display.println("% ");
  }
  else
  {
    if (!running && opMode == AUTO_MODE)
    {
      leftTime = calcTimeMinutes(leftPoti);
      rightTime = calcTimeMinutes(rightPoti);
      int speedPercent = calcSpeedPercentage(speedPoti);
      display.setTextSize(2);
      display.setCursor(0, 0);
      display.print("V ");
      display.print(speedPercent);
      display.println("% ");
      showTimeLEDs(leftTime, rightTime);
    }
    else if (running && opMode == AUTO_MODE)
    {
      int timeNeededforSlowStart = firstRunSpeedPercent * speedTimerIntervall / 60.0 / 1000.0 * 2;
      if (currentRun == 1)
      {
        timeNeededforSlowStart = secondRunSpeedPercent * speedTimerIntervall / 60.0 / 1000.0 * 2;
      }
      else if (currentRun == 2)
      {
        timeNeededforSlowStart = thirdRunSpeedPercent * speedTimerIntervall / 60.0 / 1000.0 * 2;
      }
      if (currentDirection == LEFT)
      {
        leftTime = max(calcTimeMinutes(leftPoti, 10) - (millis() - runStartTime) / 60.0 / 1000.0 + timeNeededforSlowStart, 0.0);
        rightTime = calcTimeMinutes(rightPoti, 10);
      }
      else
      {
        leftTime = calcTimeMinutes(leftPoti, 10);
        rightTime = max(calcTimeMinutes(rightPoti) - (millis() - runStartTime) / 60.0 / 1000.0 + timeNeededforSlowStart, 0.0);
      }
      int speedPercent = currentSpeed;

      display.setTextSize(2);
      display.setCursor(0, 0);
      display.print("A");
      display.print(currentRun + 1);
      if (currentDirection == LEFT)
      {
        display.print("L");
      }
      else
      {
        display.print("R");
      }
      display.print(" ");
      display.print(speedPercent);
      display.println("% ");
    }
    display.setTextSize(3);
    display.print("L ");
    display.print(leftTime, 1);
    display.println("m");
    display.print("R ");
    display.print(rightTime, 1);
    display.println("m");
  }
  display.display();
}

void startAutoRun()
{
  if (!running)
  {
    running = true;
    runStartTime = millis();
    currentRun = 0;
    currentDirection = LEFT;
    targetSpeed = firstRunSpeedPercent;
  }
}

void setup()
{
  // Disable Serial, so we can use Pin 0 and 1
  Serial.end();

  // Speed
  analogWrite(PIN_SPEED, 255); // Set Speed to Zero
  pinMode(PIN_L, OUTPUT);
  pinMode(PIN_R, OUTPUT);
  digitalWrite(PIN_L, 0); //Turn off Left
  digitalWrite(PIN_R, 0); //Turn off Right

  // // LED Pins
  pinMode(PIN_LED_R1, OUTPUT);
  pinMode(PIN_LED_R2, OUTPUT);
  pinMode(PIN_LED_R3, OUTPUT);
  pinMode(PIN_LED_R4, OUTPUT);
  pinMode(PIN_LED_R5, OUTPUT);
  pinMode(PIN_LED_L1, OUTPUT);
  pinMode(PIN_LED_L2, OUTPUT);
  pinMode(PIN_LED_L3, OUTPUT);
  pinMode(PIN_LED_L4, OUTPUT);
  pinMode(PIN_LED_L5, OUTPUT);
  digitalWrite(PIN_LED_R1, 0);
  digitalWrite(PIN_LED_R2, 0);
  digitalWrite(PIN_LED_R3, 0);
  digitalWrite(PIN_LED_R4, 0);
  digitalWrite(PIN_LED_R5, 0);
  digitalWrite(PIN_LED_L1, 0);
  digitalWrite(PIN_LED_L2, 0);
  digitalWrite(PIN_LED_L3, 0);
  digitalWrite(PIN_LED_L4, 0);
  digitalWrite(PIN_LED_L5, 0);

  // Start Switch
  pinMode(PIN_START, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_START), button_interrupt_handler, FALLING);
  initialisation_complete = true;

  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.clearDisplay();
  display.display();
}

void setSpeed()
{
  if (currentSpeed < targetSpeed)
  {
    currentSpeed++;
  }
  else if (currentSpeed > targetSpeed)
  {
    currentSpeed--;
  }
  setSpeedTimer = millis();
  analogWrite(PIN_SPEED, 255 - currentSpeed * 255 / 100);
}

void loop()
{
  int opModeSwitch = analogRead(A3);
  bool stopping = false;
  int oldMode = opMode;
  if (opModeSwitch > 682)
  {
    opMode = MANUAL_LEFT;
  }
  else if (opModeSwitch < 341)
  {
    opMode = AUTO_MODE;
  }
  else if (opModeSwitch <= 682 && opModeSwitch >= 341)
  {
    opMode = MANUAL_RIGHT;
  }

  if (opMode != oldMode && currentSpeed != 0)
  {
    stopping = true;
    opMode = oldMode;
    targetSpeed = 0;
  }

  if (!stopping && !running && opMode == AUTO_MODE)
  {
    targetSpeed = 0;
    leftPoti = analogRead(A1);
    rightPoti = analogRead(A2);
    speedPoti = potiMax - analogRead(A0);

    if (read_button() == switched)
    {
      startAutoRun();
    }
    if (currentSpeed == 0)
    {
      digitalWrite(PIN_L, 0);
      digitalWrite(PIN_R, 0);
    }
  }
  else if (!stopping && running && opMode == AUTO_MODE)
  {

    if (currentDirection == LEFT)
    {
      if (millis() - runStartTime > calcTimeMinutes(leftPoti) * 60 * 1000 + (currentRun + 1) * 33 * speedTimerIntervall)

      {
        targetSpeed = 0;
        if (currentSpeed == 0)
        {
          switch (currentRun)
          {
          case 0:
            targetSpeed = firstRunSpeedPercent;
            currentDirection = RIGHT;
            runStartTime = millis();
            break;
          case 1:
            targetSpeed = secondRunSpeedPercent;
            currentDirection = RIGHT;
            runStartTime = millis();
            break;
          case 2:
            targetSpeed = thirdRunSpeedPercent;
            currentDirection = RIGHT;
            runStartTime = millis();
            break;
          default:
            break;
          }
        }
      }
    }
    else if (currentDirection == RIGHT)
    {
      if (millis() - runStartTime > calcTimeMinutes(rightPoti) * 60 * 1000 + (currentRun + 1) * 33 * speedTimerIntervall)
      {
        targetSpeed = 0;
        if (currentSpeed == 0)
        {
          switch (currentRun)
          {
          case 0:
            currentRun = 1;
            targetSpeed = secondRunSpeedPercent;
            currentDirection = LEFT;
            runStartTime = millis();

            break;
          case 1:
            currentRun = 2;
            targetSpeed = thirdRunSpeedPercent;
            currentDirection = LEFT;
            runStartTime = millis();

            break;
          case 2:
            running = false;
            currentRun = 0;
            currentDirection = 0;
            break;
          default:
            break;
          }
        }
      }
    }
    if (currentDirection == LEFT)
    {
      digitalWrite(PIN_L, 1);
      digitalWrite(PIN_R, 0);
    }
    else if (currentDirection == RIGHT)
    {
      digitalWrite(PIN_L, 0);
      digitalWrite(PIN_R, 1);
    }
    else if (currentDirection == 0)
    {
      digitalWrite(PIN_L, 0);
      digitalWrite(PIN_R, 0);
    }
  }

  if (!stopping && (opMode == MANUAL_RIGHT || opMode == MANUAL_LEFT))
  {
    targetSpeed = calcSpeedPercentage(1023 - analogRead(A0));
    if (opMode == MANUAL_LEFT && currentSpeed == 0)
    {
      digitalWrite(PIN_L, 1);
      digitalWrite(PIN_R, 0);
    }
    if (opMode == MANUAL_RIGHT && currentSpeed == 0)
    {
      digitalWrite(PIN_L, 0);
      digitalWrite(PIN_R, 1);
    }
  }

  if (millis() - setSpeedTimer > speedTimerIntervall)
  {
    setSpeed();
  }

  showInfo();
}
