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
  answer = 0;
  digitsEntered = -1;
  lastSwitchState = RotaryEncoder::SwitchState::SW_OFF;
  rotaryEncoder.setPosition(0);
  isChanged = false;
}

int AnswerInput::getEditAnswer() {
  return answer * 10 + rotaryEncoder.getPosition();
}

int AnswerInput::getEnteredAnswer() {
  return answer;
}

int AnswerInput::getDigitsEntered() {
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

  // Set digitsEntered to zero to indicate that an answer is being edited.
  if (digitsEntered == -1) {
    digitsEntered = 0;
  }
  isChanged = true;
}

void AnswerInput::rotaryPressedCallback() {
  rotaryEncoder.switchUpdate();
  RotaryEncoder::SwitchState currentSwitchState = (RotaryEncoder::SwitchState)rotaryEncoder.getSwitchState();
  if (lastSwitchState != currentSwitchState) {
    lastSwitchState = currentSwitchState;

    if (currentSwitchState == rotaryEncoder.SW_ON) {
      answer = getEditAnswer();
      digitsEntered++;

      // Reset position to zero for next entry
      rotaryEncoder.setPosition(0);
    }
    isChanged = true;
  }
}

bool AnswerInput::getAndResetUpdate() {
  noInterrupts();

  bool result = isChanged;
  isChanged = false;

  interrupts();
  return result;
}
