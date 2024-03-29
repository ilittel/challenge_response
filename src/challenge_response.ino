#include <TM1637Display.h>
#include "RotaryInput.h"
#include <LowPower.h>
#include <avr/sleep.h>

const byte ROTARY_PIN_A = 3;
const byte ROTARY_PIN_B = 4;
const byte ROTARY_PIN_BUTTON = 2;

const uint8_t CLK = 7;
const uint8_t DIO = 8;
const uint8_t LED_R = 9;
const uint8_t LED_G = 6;
const uint8_t LED_B = 5;
const uint8_t SOLENOID_PIN = 10;

const float REFERENCE_VOLTAGE = 1.107;
const float VOLTAGE_DIVIDER_FACTOR = ((100.0 + 10.0) / 10.0);

const float CHARGE_THRESHOLD_YELLOW = 4.5;
const float CHARGE_THRESHOLD_GREEN = 5.0;

const float DISCHARGE_THRESHOLD_YELLOW = 4.5;
const float DISCHARGE_THRESHOLD_RED = 3.5;

int lastAnswer = -1;
uint8_t lastDigitsEntered = 0;
bool timerProcessed = false;

enum ProgramState {
  STATE_UNINITIALIZED,
  STATE_CHARGING,
  STATE_DISPLAYING_CHALLENGE,
  STATE_ENTERING_RESPONSE,
  STATE_PROCESSING_RESPONSE
};

enum PowerState {
  GREEN,
  YELLOW,
  RED
};

TM1637Display display = TM1637Display(CLK, DIO);
RotaryInput& rotaryInput = RotaryInput::init(ROTARY_PIN_A, ROTARY_PIN_B, ROTARY_PIN_BUTTON);

unsigned int challenge = 0;
unsigned int correctAnswer = 0;
PowerState powerState = RED;

volatile ProgramState programState = STATE_UNINITIALIZED;

uint8_t INITIAL_SEGMENTS[4] = { SEG_G, SEG_G, SEG_G, SEG_G };

void setup() {
  // TODO: Remove all serial comms after debugging.
  // initialize serial communication at 9600 bits per second:  
  Serial.begin(9600);

  // Set up display
  display.setBrightness(1);

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
// Updates the global program state.
// 
// Currently this function is not atomic, so it may only be called from the main loop!
// 
void updateProgramState() {
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
        setProgramState(STATE_PROCESSING_RESPONSE);
      }
    break;
    // TODO: Kan deze state niet weg? De transitie kan ook gelijk hierboven plaatsvinden.
    case STATE_PROCESSING_RESPONSE:
      if (lastAnswer == correctAnswer) {
        activateSolenoid();
        powerState = RED; // Make sure the solenoid cap is recharged.
        setProgramState(STATE_CHARGING);
      } else {
        setProgramState(STATE_DISPLAYING_CHALLENGE);
      }
    break;
    default:
      Serial.print("Error: unhandled state value: ");
      Serial.println(programState);
      Serial.flush();
    break;
  }
}

void setProgramState(ProgramState newState) {
  switch(newState) {
    case STATE_CHARGING:
      challenge = 4321; // TODO
      correctAnswer = 1234; // TODO
      display.clear();
    break;
    case STATE_DISPLAYING_CHALLENGE:
      // Reset rotary input to prevent immediate transition to 'entering response' state.
      rotaryInput.reset();
      display.showNumberDec(challenge);
    break;
    case STATE_ENTERING_RESPONSE:
      // Do nothing
    break;
    case STATE_PROCESSING_RESPONSE:
      // Do nothing
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

void showAnswer() {
  if (programState == STATE_ENTERING_RESPONSE) {
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
}

void activateSolenoid() {
  Serial.println("Activating solenoid");
  Serial.flush();

  digitalWrite(SOLENOID_PIN, HIGH);  
  delay(1000);
  digitalWrite(SOLENOID_PIN, LOW);
}
