#include <Arduino.h>
#include "challenge_response.h"

#include <TM1637Display.h>
#include "RotaryInput.h"
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

const int PUZZLE_SOLUTION_ABCD = 7828;

const float REFERENCE_VOLTAGE = 1.107;
const float VOLTAGE_DIVIDER_FACTOR = ((100.0 + 10.0) / 10.0);

const float CHARGE_THRESHOLD_YELLOW = 5.5; // TODO: re-determine after finishing final algorithm
const float CHARGE_THRESHOLD_GREEN = 7.0; // TODO: re-determine after finishing final algorithm

const float DISCHARGE_THRESHOLD_YELLOW = 5.0; // TODO: re-determine after finishing final algorithm
const float DISCHARGE_THRESHOLD_RED = 4.5; // TODO: re-determine after finishing final algorithm

int lastAnswer = -1;
uint8_t lastDigitsEntered = 0;
bool timerProcessed = false;

TM1637Display display = TM1637Display(CLK, DIO);
RotaryInput& rotaryInput = RotaryInput::init(ROTARY_PIN_A, ROTARY_PIN_B, ROTARY_PIN_BUTTON);

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
  // TODO: Remove all serial comms after debugging.
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
  noInterrupts();
  bool watchDogTimerEnabled = watchDogTimerOn();
  interrupts();
  if (!watchDogTimerEnabled && !timerProcessed) { // Loop was re-entered because of sleep timeout and it wasn't processed yet.
    updatePowerIndicator();
    timerProcessed = true;
  }

  updatePowerState();
  updateProgramState();

  noInterrupts();
  // Check if input has changed.
  if (rotaryInput.getAnswer() != lastAnswer ||
      rotaryInput.getDigitsEntered() != lastDigitsEntered) {
    // Store answer + digits entered into variables and use those everywhere else so we
    // don't have the risk of suddenly updated values.
    lastAnswer = rotaryInput.getAnswer();
    lastDigitsEntered = rotaryInput.getDigitsEntered();

    interrupts();
    // (After this point, loop() is re-entered to process changes and see if other changes were made; 
    // otherwise, go to sleep (again).)
  } else { // input hasn't changed -> go to sleep.
    if (watchDogTimerOn()) { // Last sleep period isn't over yet.
      sleep_enable();
      interrupts();
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

bool watchDogTimerOn() {
  return (WDTCSR & (1<<WDIE));
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
  // Serial.print("programState = ");
  // Serial.println(programState);
  // Serial.print("lastAnswer = ");
  // Serial.println(lastAnswer);
  // Serial.print("lastDigitsEntered = ");
  // Serial.println(lastDigitsEntered);
  // Serial.flush();

  switch (programState) {
    case STATE_CHARGING:
      if (powerState == GREEN) {
        setProgramState(STATE_DISPLAYING_CHALLENGE);
      }
    break;
    case STATE_DISPLAYING_CHALLENGE:
      if (lastAnswer > -1) {
        setProgramState(STATE_ENTERING_RESPONSE);
      }
    break;
    case STATE_ENTERING_RESPONSE:
      showAnswer();
      if (lastDigitsEntered > 3) {
        if (lastAnswer == correctAnswer) {
          setProgramState(STATE_ANSWERED_CORRECTLY);
        } else {
          setProgramState(STATE_ANSWERED_WRONGLY);
        }
      }
    break;
    case STATE_ANSWERED_WRONGLY:
      setProgramState(STATE_DISPLAYING_CHALLENGE);
    break;
    case STATE_ANSWERED_CORRECTLY:
      // Do nothing
    break;
    default:
      Serial.print("Error: unhandled state value: ");
      Serial.println(programState);
      Serial.flush();
    break;
  }
}

//
// Implements state transition logic.
// 
void setProgramState(ProgramState newState) {
  switch(newState) {
    case STATE_CHARGING:
      display.clear();
    break;
    case STATE_DISPLAYING_CHALLENGE:
      resetChallenge();
      // Reset rotary input to prevent immediate transition to 'entering response' state.
      rotaryInput.reset();
      display.showNumberDec(challenge, true);
    break;
    case STATE_ENTERING_RESPONSE:
      // Do nothing
    break;
    case STATE_ANSWERED_WRONGLY:
      displayError();
    break;
    case STATE_ANSWERED_CORRECTLY:
      activateSolenoid();
      // After activation, blink the RGB LED until we are out of power.
      blinkTillTheEnd();
    break;
    default:
      Serial.print("Error: invalid new state value: ");
      Serial.println(newState);
      Serial.flush();
      return;
    break;
  }

  programState = newState;
}

void resetChallenge() {
  randomSeed((unsigned long)(analogRead(A0) + analogRead(A1)));
  challenge = (int)random(0, 9999);
  correctAnswer = calculateAnswer();
}

int calculateAnswer() {
  return (PUZZLE_SOLUTION_ABCD ^ challenge) % 10000;
}

void showAnswer() {
  uint8_t segments[4];

  // Show between 1 and 4 digits, as we also need to show the digit that is being entered.
  const uint8_t digitsToShow = min(lastDigitsEntered + 1, 4);

  int partialNumber = lastAnswer;
  int index = 3;
  while (index >= 4 - digitsToShow) {
    uint8_t digit = (uint8_t)(partialNumber % 10);
    segments[index] = display.encodeDigit(digit);

    partialNumber /= 10;
    index--;
  }

  // Set underscores to segments before
  while (index >= 0) {
    segments[index] = SEG_D;
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

void blinkTillTheEnd() {
  while (true) {
    analogWrite(LED_R, (int)random(0, 25));
    analogWrite(LED_G, (int)random(0, 25));
    analogWrite(LED_B, (int)random(0, 25));

    delay(100);
  }
}
