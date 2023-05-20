#include <TM1637Display.h>
#include <FR_RotaryEncoder.h>
#include <LowPower.h>

const byte ROTARY_PIN_A = 3;
const byte ROTARY_PIN_B = 4;
const byte ROTARY_PIN_BUTTON = 2;

const uint8_t CLK = 7;
const uint8_t DIO = 8;
const uint8_t LED_B = 5;
const uint8_t LED_G = 6;
const uint8_t LED_R = 9;

const float VOLTAGE_DIVIDER_FACTOR = ((100.0 + 10.0) / 10.0);

const float CHARGE_THRESHOLD_YELLOW = 4.5;
const float CHARGE_THRESHOLD_GREEN = 5.1;

const float DISCHARGE_THRESHOLD_YELLOW = 4.0;
const float DISCHARGE_THRESHOLD_RED = 3.5;

enum ProgramState {
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
RotaryEncoder rotaryEncoder(ROTARY_PIN_A, ROTARY_PIN_B, ROTARY_PIN_BUTTON);

PowerState powerState;
unsigned int challenge;
unsigned int correctAnswer;

volatile ProgramState programState;
volatile unsigned int digitsEntered;
volatile unsigned int answer;
volatile unsigned int lastDigitValue;
volatile RotaryEncoder::SwitchState lastSwitchState;

uint8_t INITIAL_SEGMENTS[4] = { SEG_G, SEG_G, SEG_G, SEG_G };

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600); // TODO: Remove after debugging

  // Set up display
  display.clear();  
  display.setBrightness(7);

  // Set up voltage reading
  analogReference(INTERNAL);
  pinMode(A0, INPUT);

  // Set up RGB LED
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  pinMode(ROTARY_PIN_A, INPUT_PULLUP);
  pinMode(ROTARY_PIN_B, INPUT_PULLUP);
  pinMode(ROTARY_PIN_BUTTON, INPUT_PULLUP);

  // Set up rotary encoder
  rotaryEncoder.enableInternalRotaryPullups();
  rotaryEncoder.enableInternalSwitchPullup();
  rotaryEncoder.setRotaryLimits(0, 9, false);
  //rotaryEncoder.setSwitchDebounceDelay(5);

  attachInterrupt(digitalPinToInterrupt(ROTARY_PIN_A), rotaryChangeCallback, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROTARY_PIN_BUTTON), rotaryPressCallback, CHANGE);

  // Initialize power and program state
  powerState = RED;
  programState = STATE_CHARGING;

  challenge = 4321; // TODO
  correctAnswer = 1234; // TODO

  digitsEntered = 0;
  answer = 0;
  lastDigitValue = 0;
  lastSwitchState = rotaryEncoder.SW_OFF;
}

void rotaryChangeCallback() {
  rotaryEncoder.rotaryUpdate();

  if (programState == STATE_ENTERING_RESPONSE) {
    unsigned int currentPosition = (unsigned int)rotaryEncoder.getPosition();
    if (lastDigitValue != currentPosition) {
      lastDigitValue = currentPosition;

      int direction = rotaryEncoder.getDirection();

      // TODO: Remove after debugging
      Serial.print("Rotary updated, position: ");
      Serial.print(currentPosition);
      Serial.print(", direction: ");
      printRotationalDirection(direction);
      Serial.println();
      Serial.flush();

      // Replace last digit of current answer with rotary's position.
      answer = (answer / 10)*10 + lastDigitValue;

      display.showNumberDec(answer, true);
    }
  }
}

// TODO: Remove after debugging
void printRotationalDirection(int direction) {
  switch(direction) {
    case rotaryEncoder.CW:
      Serial.print("CW");
      break;
    case rotaryEncoder.CCW:
      Serial.print("CCW");
      break;
    case rotaryEncoder.NOT_MOVED:
      Serial.print("NOT_MOVED");
      break;
    default:
      Serial.print("Unrecognized direction of rotation");  
  } 
}

void rotaryPressCallback() {
  rotaryEncoder.switchUpdate();

  if (programState == STATE_ENTERING_RESPONSE) {
    RotaryEncoder::SwitchState currentSwitchState = (RotaryEncoder::SwitchState)rotaryEncoder.getSwitchState();
    if (lastSwitchState != currentSwitchState) {
      lastSwitchState = currentSwitchState;

      if (currentSwitchState == rotaryEncoder.SW_ON) {
        Serial.println("Rotary pressed");

        int position = rotaryEncoder.getPosition();
        digitsEntered++;
        if (digitsEntered < 4) {
          answer *= 10;
        }

        rotaryEncoder.setPosition(0);
        display.showNumberDec(answer, true);
      } else if (currentSwitchState == rotaryEncoder.SW_OFF) {
        Serial.println("Rotary unpressed");
      }

      Serial.flush();
    }
  }
}

void loop() {
  float voltage = readVoltage();

  updatePowerState(voltage);

  updateProgramState(voltage);

  // TODO: Remove after debugging
  // Serial.print("Measured voltage: ");
  // Serial.println(voltage);
  // Serial.print("Rotary position: ");
  // Serial.println(rotaryEncoder.getPosition());
  // Serial.flush();
  
  delay(1000);
  //LowPower.powerDown(SLEEP_1S, ADC_ON, BOD_OFF);
}

float readVoltage() {
  // read the input on analog pin 0:
  int sensorValue = analogRead(A0);
  
  // TODO: Remove after debugging
  // Serial.print("Raw A0 value: ");
  // Serial.println(sensorValue);
  // Serial.flush();

  // Convert the analog reading (which goes from 0 - 1023) to the reference voltage (0 - 1.1V) and compensate for the voltage divider.
  return sensorValue * (1.1 / 1023.0) * VOLTAGE_DIVIDER_FACTOR;
}

void updatePowerState(float voltage) {
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

  updatePowerIndicator();
}

void updatePowerIndicator() {
  switch (powerState) {
    case RED:
      analogWrite(LED_R, 1);
      analogWrite(LED_G, 0);
      analogWrite(LED_B, 0);
    break;
    case YELLOW:
      analogWrite(LED_R, 1);
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
}

void updateProgramState(float voltage) {
  switch (programState) {
    case STATE_CHARGING:
      if (voltage > CHARGE_THRESHOLD_GREEN) {
        setProgramState(STATE_DISPLAYING_CHALLENGE);
      }
    break;
    case STATE_DISPLAYING_CHALLENGE:
      if (rotaryEncoder.getPosition() > 0) {
        setProgramState(STATE_ENTERING_RESPONSE);
      }
    break;
    case STATE_ENTERING_RESPONSE:
      if (digitsEntered > 3) {
        setProgramState(STATE_PROCESSING_RESPONSE);
      }
    break;
    case STATE_PROCESSING_RESPONSE:
      if (answer == correctAnswer) {
        activateSolenoid();
      } else {
        setProgramState(STATE_ENTERING_RESPONSE);
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
    case STATE_DISPLAYING_CHALLENGE:
      display.showNumberDec(challenge);
    break;
    case STATE_ENTERING_RESPONSE:
      digitsEntered = 0;
      answer = 0;
      display.setSegments(INITIAL_SEGMENTS);
    break;
    case STATE_PROCESSING_RESPONSE:
      // Do nothing
    break;
    default:
      Serial.print("Error: invalid new state value: ");
      Serial.println(newState);
      return;
    break;
  }

  programState = newState;  
}

void activateSolenoid() {
  Serial.println("Activating solenoid");
  Serial.flush();
  // TODO
}
