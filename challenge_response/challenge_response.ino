#include <TM1637Display.h>
#include <ec11.hpp>
#include <LowPower.h>

using namespace a21;

const byte ROTARY_PIN_A = 2;
const byte ROTARY_PIN_B = 3;
const byte ROTARY_PIN_BUTTON = 12;

const byte ROTARY_CLICKS_PER_STEP = 4;

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
 STATE_ACTIVATING_SOLENOID,
 STATE_DISPLAYING_ERROR,
 STATE_DISPLAYING_UNLOCKED
};

enum PowerState {
  GREEN,
  YELLOW,
  RED
};

TM1637Display display = TM1637Display(CLK, DIO);
EC11 rotaryEncoder;

ProgramState state;
PowerState powerState;
int answer;
uint8_t INITIAL_SEGMENTS[4] = { SEG_G, SEG_G, SEG_G, SEG_G };

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600); // TODO: Remove after debugging

  // Set up display
  display.clear();  
  display.setBrightness(7);

  display.setSegments(INITIAL_SEGMENTS);

  // Set up voltage reading
  analogReference(INTERNAL);
  pinMode(A0, INPUT);

  // Set up RGB LED
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  pinMode(ROTARY_PIN_A, INPUT_PULLUP);
  pinMode(ROTARY_PIN_B, INPUT_PULLUP);

  // Set up rotary encoder
  attachInterrupt(digitalPinToInterrupt(ROTARY_PIN_A), rotaryChangeCallback, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROTARY_PIN_B), rotaryChangeCallback, CHANGE);

  // Initialize program and power state
  state = STATE_CHARGING;
  powerState = RED;
  answer = 0;
}

void rotaryChangeCallback() {
  rotaryEncoder.checkPins(digitalRead(ROTARY_PIN_A), digitalRead(ROTARY_PIN_B));
}

void rotaryPressCallback() {
  Serial.println("Rotary pressed");
  answer *= 10;

  display.showNumberDec(answer, true);
}

void loop() {
  float voltage = readVoltage();

  processState(voltage);

  // TODO: Remove after debugging
  Serial.print("Measured voltage: ");
  Serial.println(voltage);
  Serial.flush();
  
  delay(1000);

  EC11Event e;
  if (rotaryEncoder.read(&e)) {

    // OK, got an event waiting to be handled.
    
    if (e.type == EC11Event::StepCW) {
      answer += e.count;
    } else {
      answer -= e.count;
    }
    Serial.print("Answer changed, new value: ");
    Serial.println(answer);
    display.showNumberDec(answer, true);
  }    

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

void processState(float voltage) {
  setPowerState(voltage);

  switch (powerState) {
    case STATE_CHARGING:
      if (voltage > CHARGE_THRESHOLD_GREEN) {
        display.showNumberDec(0, true);
        state = STATE_ENTERING_RESPONSE;
      }
    break;
    case STATE_ENTERING_RESPONSE:
      // Hier gebleven
    break;
    default:
    break;
  }

  updatePowerIndicator();
}

void setPowerState(float voltage) {
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
      Serial.println("ERROR: Invalid state value in updatePowerIndicator()");
      Serial.flush();
    break;
  }
}
