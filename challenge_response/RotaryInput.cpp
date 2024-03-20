#include "RotaryInput.h"

RotaryInput* RotaryInput::INSTANCE = NULL;

RotaryInput::RotaryInput(byte pinA, byte pinB, byte pinButton)
  : rotaryEncoder(pinA, pinB, pinButton) {
  this->pinA = pinA;
  this->pinButton = pinButton;
 
  rotaryEncoder.enableInternalRotaryPullups();
  rotaryEncoder.enableInternalSwitchPullup();
  rotaryEncoder.setRotaryLimits(0, 9, false);
  rotaryEncoder.setSwitchDebounceDelay(0); // Debounce doesn't seem to be required and misses clicks if >5ms
  reset();
}

static RotaryInput& RotaryInput::init(byte pinA, byte pinB, byte pinButton) {
  if (INSTANCE != NULL) {
    detachInterrupt(digitalPinToInterrupt(INSTANCE->pinA));
    detachInterrupt(digitalPinToInterrupt(INSTANCE->pinButton));
    delete INSTANCE;
  }
  
  INSTANCE = new RotaryInput(pinA, pinB, pinButton);

  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  pinMode(pinButton, INPUT_PULLUP);

  // TODO: Don't we need to attach pinB??
  attachInterrupt(digitalPinToInterrupt(pinA), CHANGED_CALLBACK, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinButton), PRESSED_CALLBACK, CHANGE);

  return *INSTANCE;
}

void RotaryInput::reset() {
  partialAnswer = 0;
  digitsEntered = 0;
  lastSwitchState = RotaryEncoder::SwitchState::SW_OFF;
  rotaryEncoder.setPosition(-1);
}

int RotaryInput::getAnswer() {
  return partialAnswer + rotaryEncoder.getPosition();
}

uint8_t RotaryInput::getDigitsEntered() {
  return digitsEntered;
}

static void RotaryInput::CHANGED_CALLBACK() {
  INSTANCE->rotaryChangedCallback();
}

static void RotaryInput::PRESSED_CALLBACK() {
  INSTANCE->rotaryPressedCallback();
}

void RotaryInput::rotaryChangedCallback() {
  rotaryEncoder.rotaryUpdate();
  nrChangedInterrupts++;
}

void RotaryInput::rotaryPressedCallback() {
  rotaryEncoder.switchUpdate();
  nrPressedInterrupts++;
  RotaryEncoder::SwitchState currentSwitchState = (RotaryEncoder::SwitchState)rotaryEncoder.getSwitchState();
  if (lastSwitchState != currentSwitchState) {
    lastSwitchState = currentSwitchState;

    int position = rotaryEncoder.getPosition();
    if (position > -1 && currentSwitchState == rotaryEncoder.SW_ON) {
      if (digitsEntered < 3) {
        partialAnswer = (partialAnswer + position) * 10;
        rotaryEncoder.setPosition(0);
      }

      if (digitsEntered < 4) {
        digitsEntered++;
      }
    }
  }
}
