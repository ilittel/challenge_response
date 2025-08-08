#include <Arduino.h>
#include "challenge_response.h"

#include <TM1637Display.h>
#include <LowPower.h>
#include <avr/sleep.h>

const byte ROTARY_PIN_BUTTON = 2; // SW
const byte ROTARY_PIN_A = 3;      // CLK
const byte ROTARY_PIN_B = 4;      // DT

const uint8_t CLK = 8;
const uint8_t DIO = 7;
const uint8_t LED_R = 9;
const uint8_t LED_G = 6;
const uint8_t LED_B = 5;
const uint8_t SOLENOID_PIN = 10;

const float REFERENCE_VOLTAGE = 1.107;
const float VOLTAGE_DIVIDER_FACTOR = ((100.0 + 10.0) / 10.0);

const float CHARGE_THRESHOLD_YELLOW = 5.5;
const float CHARGE_THRESHOLD_GREEN = 7.0;

const float DISCHARGE_THRESHOLD_YELLOW = 5.0;
const float DISCHARGE_THRESHOLD_RED = 4.5;

bool timerProcessed = false;

TM1637Display display = TM1637Display(CLK, DIO);
AnswerInput& answerInput = AnswerInput::init(ROTARY_PIN_A, ROTARY_PIN_B, ROTARY_PIN_BUTTON);

int challenge = 0;
int correctAnswer = 0;
PowerState powerState = RED;

volatile ProgramState programState = STATE_UNINITIALIZED;

uint8_t INITIAL_SEGMENTS[4] = { SEG_G, SEG_G, SEG_G, SEG_G };
uint8_t ERROR_SEGMENTS[4] = { SEG_A | SEG_D | SEG_E | SEG_F | SEG_G, 
                              SEG_A | SEG_E | SEG_F, 
                              SEG_A | SEG_E | SEG_F, 
                              0 };

void setup() {
  // initialize serial communication at 9600 bits per second:  
  Serial.begin(9600);

  // Set up display
  display.setBrightness(5);

  // Set up voltage reading
  analogReference(INTERNAL);
  pinMode(A0, INPUT);

  // Set up RGB LED
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(SOLENOID_PIN, OUTPUT);
  digitalWrite(SOLENOID_PIN, LOW);

  setProgramState(STATE_CHARGING);
}

void loop() {
  if (!isWatchDogTimerOn() && !timerProcessed) { // Loop was re-entered because of sleep timeout and it wasn't processed yet.
    updatePowerIndicator();
    timerProcessed = true;
  }

  update();

  // Check if input has changed.
  if (answerInput.getAndResetUpdate()) {
    // (After this point, loop() is re-entered to process changes and see if other changes were made; 
    // otherwise, go to sleep (again).)
  } else { // input hasn't changed -> go to sleep.
    if (isWatchDogTimerOn()) { // Last sleep period isn't over yet.
      sleep_enable();
      sleep_cpu();
      // (At this point, sleep was interrupted again by pin interrupt or sleep timeout.)
      sleep_disable();
    } else { // Watchdog timed out -> enter a new sleep period.
      interrupts();
      timerProcessed = false;
      LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
      // (At this point, sleep was interrupted by pin interrupt or sleep timeout.)
    }
  }
}

bool isWatchDogTimerOn() {
  noInterrupts();
  bool result = (WDTCSR & (1<<WDIE));
  interrupts();

  return result;
}

void update() {
  updatePowerState();
  updateProgramState();
  updateOutput();
}

void updatePowerState() {
  float voltage = readVoltage();

  // Serial.print("Measured voltage: ");
  // Serial.print(voltage);
  // Serial.println();
  // Serial.flush();
  switch (powerState) {
    case RED:
      if (voltage > CHARGE_THRESHOLD_YELLOW) {
        powerState = YELLOW;
      }
    break;
    case YELLOW:
      if (voltage > CHARGE_THRESHOLD_GREEN) {
        powerState = GREEN;
      } else if (voltage < DISCHARGE_THRESHOLD_RED) {
        powerState = RED;
      }
    break;
    case GREEN:
      if (voltage < DISCHARGE_THRESHOLD_YELLOW) {
        powerState = YELLOW;
      }
    break;
    default:
    break;
  }
}

float readVoltage() {
  // read the input on analog pin 0:
  int sensorValue = analogRead(A0);
  
  // Convert the analog reading (which goes from 0 - 1023) to the reference voltage (0 - 1.1V) and compensate for the voltage divider.
  return sensorValue * (REFERENCE_VOLTAGE / 1023.0) * VOLTAGE_DIVIDER_FACTOR;
}

void updatePowerIndicator() {
  switch (powerState) {
    case RED:
      analogWrite(LED_R, 10);
      analogWrite(LED_G, 0);
      analogWrite(LED_B, 0);
    break;
    case YELLOW:
      analogWrite(LED_R, 10);
      analogWrite(LED_G, 1);
      analogWrite(LED_B, 0);
    break;
    case GREEN:
      analogWrite(LED_R, 0);
      analogWrite(LED_G, 1);
      analogWrite(LED_B, 0);
    break;
    default:
      Serial.print("ERROR: unhandled power state value: ");
      Serial.println(powerState);
      Serial.flush();
    break;
  }

  delay(50);

  analogWrite(LED_R, 0);
  analogWrite(LED_G, 0);
  analogWrite(LED_B, 0);
}

//
// Implements decision logic for updating global program state.
// 
void updateProgramState() {
  switch (programState) {
    case STATE_CHARGING:
      if (powerState == GREEN) {
        resetChallenge();
        // Resetting the answer input also prevents immediate transition to 'entering response' state.
        answerInput.reset();
        setProgramState(STATE_DISPLAYING_CHALLENGE);
      }
    break;
    case STATE_DISPLAYING_CHALLENGE:
      if (answerInput.getDigitsEntered() != -1) {
        setProgramState(STATE_ENTERING_RESPONSE);
      }
    break;
    case STATE_ENTERING_RESPONSE:
      if (answerInput.getDigitsEntered() == 4) {
        if (answerInput.getEnteredAnswer() == correctAnswer) {
          setProgramState(STATE_ANSWERED_CORRECTLY);
        } else {
          setProgramState(STATE_ANSWERED_WRONGLY);
        }
      }
    break;
    case STATE_ANSWERED_WRONGLY:
      // Resetting the answer input also prevents immediate transition to 'entering response' state.
      answerInput.reset();
      setProgramState(STATE_DISPLAYING_CHALLENGE);
    break;
    case STATE_ANSWERED_CORRECTLY:
      setProgramState(STATE_CHARGING);
    break;
    default:
      Serial.print("Error: unhandled state value: ");
      Serial.println(programState);
      Serial.flush();
    break;
  }
}

void setProgramState(ProgramState newState) {
  programState = newState;
}

void updateOutput() {
  switch(programState) {
    case STATE_CHARGING:
      display.clear();
    break;
    case STATE_DISPLAYING_CHALLENGE:
      display.showNumberDec(challenge, true);
    break;
    case STATE_ENTERING_RESPONSE:
      displayCurrentAnswer();
    break;
    case STATE_ANSWERED_WRONGLY:
      displayError();
    break;
    case STATE_ANSWERED_CORRECTLY:
      activateSolenoid();
      blink();
    break;
    default:
      Serial.print("Error: invalid state value: ");
      Serial.println(programState);
      Serial.flush();
    break;
  }
}

void resetChallenge() {
  randomSeed((unsigned long)(analogRead(A0) + analogRead(A1)));
  challenge = (int)random(0, 9999);
  correctAnswer = calculateAnswer();
}

int calculateAnswer() {
  return (int)(20250830L % challenge);
}

//
// Displays the current answer being edited on the 4-digit display. 
// Editing is from left to right, with underscores representing the numbers that are not given yet.
//
void displayCurrentAnswer() {
//
// Algorithm example:
//
// input = 20X, where X is digit being entered
// 1. Set rightmost index to underscore
// -> editIndex = 3;
// -> segments[3] = _
// 2. Fill remaining indexes with numbers, right to left
// -> partialNumber = 20X
// -> index = 2
// -> segments[2] = partialNumber % 10 = X;
// -> partialNumber /= 10 = 20;
// -> index = 1
// -> segments[1] = partialNumber % 10 = 0;
// -> partialNumber /= 10 = 2;
// -> index = 0
// -> segments[0] = partialNumber % 10 = 2;

  uint8_t segments[4];

  const uint8_t editIndex = answerInput.getDigitsEntered();

  int index = 3;
  // Start setting underscores to the rightmost segments
  while (index > editIndex) {
    segments[index] = SEG_D;
    index--;
  }

  // Then set the segments representing the number, also from right to left
  int partialNumber = answerInput.getEditAnswer();
  while (index >= 0) {
    uint8_t digit = (uint8_t)(partialNumber % 10);
    segments[index] = display.encodeDigit(digit);

    partialNumber /= 10;
    index--;
  }

  display.setSegments(segments);
}

void displayError() {
  display.setSegments(ERROR_SEGMENTS);
  delay(1000);
}

void activateSolenoid() {
  Serial.println("Activating solenoid");
  Serial.flush();

  digitalWrite(SOLENOID_PIN, HIGH);  
  delay(1000);
  digitalWrite(SOLENOID_PIN, LOW);
}

void blink() {
  // HACK: Capture the event loop and wait until the power state is no longer green. This forces the user to recharge,
  // which in turh causes the solenoid capacitor to get enough charge again.
  while (powerState == GREEN) {
    updatePowerState();
    analogWrite(LED_R, (int)random(0, 25));
    analogWrite(LED_G, (int)random(0, 25));
    analogWrite(LED_B, (int)random(0, 25));

    delay(100);
  }
}
