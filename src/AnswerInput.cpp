#include "AnswerInput.h"

AnswerInput* AnswerInput::INSTANCE = NULL;

AnswerInput::AnswerInput(byte pinA, byte pinB, byte pinButton)
  : rotaryEncoder(pinA, pinB, pinButton) {
  this->pinA = pinA;
  this->pinButton = pinButton;
 
  rotaryEncoder.enableInternalRotaryPullups();
  rotaryEncoder.enableInternalSwitchPullup();
  rotaryEncoder.setRotaryLimits(0, 9, true);
  rotaryEncoder.setSwitchDebounceDelay(0); // Debounce doesn't seem to be required and misses clicks if >5ms
  reset();
}

AnswerInput& AnswerInput::init(byte pinA, byte pinB, byte pinButton) {
  if (INSTANCE != NULL) {
    detachInterrupt(digitalPinToInterrupt(INSTANCE->pinA));
    detachInterrupt(digitalPinToInterrupt(INSTANCE->pinButton));
    delete INSTANCE;
  }
  
  INSTANCE = new AnswerInput(pinA, pinB, pinButton);

  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  pinMode(pinButton, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(pinA), CHANGED_CALLBACK, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinButton), PRESSED_CALLBACK, CHANGE);

  return *INSTANCE;
}

void AnswerInput::reset() {
  partialAnswer = 0;
  digitsEntered = 0;
  lastSwitchState = RotaryEncoder::SwitchState::SW_OFF;
  rotaryEncoder.setPosition(-1);
  isChanged = false;
}

int AnswerInput::getAnswer() {
  return partialAnswer + rotaryEncoder.getPosition();
}

uint8_t AnswerInput::getDigitsEntered() {
  return digitsEntered;
}

void AnswerInput::CHANGED_CALLBACK() {
  INSTANCE->rotaryChangedCallback();
}

void AnswerInput::PRESSED_CALLBACK() {
  INSTANCE->rotaryPressedCallback();
}

void AnswerInput::rotaryChangedCallback() {
  rotaryEncoder.rotaryUpdate();
  isChanged = true;
}

void AnswerInput::rotaryPressedCallback() {
  rotaryEncoder.switchUpdate();
  RotaryEncoder::SwitchState currentSwitchState = (RotaryEncoder::SwitchState)rotaryEncoder.getSwitchState();
  if (lastSwitchState != currentSwitchState) {
    lastSwitchState = currentSwitchState;

    if (currentSwitchState == rotaryEncoder.SW_ON) {
      int position = rotaryEncoder.getPosition();
      if (position == -1) {
        // Set position to zero to indicate that an answer is being edited
        rotaryEncoder.setPosition(0);
      } else {
        if (digitsEntered < 3) {
          partialAnswer = (partialAnswer + position) * 10;
          rotaryEncoder.setPosition(0);
        }

        if (digitsEntered < 4) {
          digitsEntered++;
        }
      }
    }
    isChanged = true;
  }
}

bool AnswerInput::readChanged() {
  noInterrupts();

  bool result = isChanged;
  isChanged = false;

  interrupts();
  return result;
}

bool AnswerInput::isFinalAnswer() {
  return digitsEntered == 4;
}
