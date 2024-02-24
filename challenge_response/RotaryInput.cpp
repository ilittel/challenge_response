#include "RotaryInput.h"

RotaryInput* RotaryInput::INSTANCE = NULL;

RotaryInput::RotaryInput(byte pinA, byte pinB, byte pinButton, void (*inputChangedCallback)())
  : rotaryEncoder(pinA, pinB, pinButton) {
  this->pinA = pinA;
  this->pinButton = pinButton;
  inputUpdatedCallback = inputChangedCallback;

  rotaryEncoder.enableInternalRotaryPullups();
  rotaryEncoder.enableInternalSwitchPullup();
  rotaryEncoder.setRotaryLimits(0, 9, false);
  rotaryEncoder.setSwitchDebounceDelay(0); // Debounce doesn't seem to be required and misses clicks if >5ms
}

static RotaryInput& RotaryInput::init(byte pinA, byte pinB, byte pinButton, void (*inputChangedCallback)()) {
  if (INSTANCE != NULL) {
    detachInterrupt(digitalPinToInterrupt(INSTANCE->pinA));
    detachInterrupt(digitalPinToInterrupt(INSTANCE->pinButton));
    delete INSTANCE;
  }
  
  INSTANCE = new RotaryInput(pinA, pinB, pinButton, inputChangedCallback);

  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  pinMode(pinButton, INPUT_PULLUP);

  // TODO: Don't we need to attach pinB??
  attachInterrupt(digitalPinToInterrupt(pinA), globalChangeCallback, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinButton), globalPressedCallback, CHANGE);

  return *INSTANCE;
}

void RotaryInput::reset() {
  digitsEntered = 0;
  answer = 0;
  lastDigitValue = 0;
  lastSwitchState = RotaryEncoder::SwitchState::SW_OFF;
}

unsigned int RotaryInput::getAnswer() {
  return answer;
}

uint8_t RotaryInput::getDigitsEntered() {
  return digitsEntered;
}

static void RotaryInput::globalChangeCallback() {
  INSTANCE->rotaryChangeCallback();
}

static void RotaryInput::globalPressedCallback() {
  INSTANCE->rotaryPressedCallback();
}

void RotaryInput::rotaryChangeCallback() {
  rotaryEncoder.rotaryUpdate();

  uint8_t currentPosition = (uint8_t)rotaryEncoder.getPosition();
  if (lastDigitValue != currentPosition) {
    lastDigitValue = currentPosition;

    // Replace last digit of current answer with rotary's position.
    answer = ((unsigned int)(answer / 10)*10) + lastDigitValue;

    inputUpdatedCallback();
  }
}

void RotaryInput::rotaryPressedCallback() {
  rotaryEncoder.switchUpdate();
  RotaryEncoder::SwitchState currentSwitchState = (RotaryEncoder::SwitchState)rotaryEncoder.getSwitchState();
  if (lastSwitchState != currentSwitchState) {
    lastSwitchState = currentSwitchState;

    if (currentSwitchState == rotaryEncoder.SW_ON) {
      if (digitsEntered < 3) {
        answer *= 10;
      }

      if (digitsEntered < 4) {
        digitsEntered++;
      }

      rotaryEncoder.setPosition(0);
      inputUpdatedCallback();
    }
  }
}

