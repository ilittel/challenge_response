#include <Arduino.h>

#include <TM1637Display.h>
#include "RotaryInput.h"
#include <LowPower.h>
#include <avr/sleep.h>

enum ProgramState {
  STATE_UNINITIALIZED,
  STATE_CHARGING,
  STATE_DISPLAYING_CHALLENGE,
  STATE_ENTERING_RESPONSE,
  STATE_ANSWERED_WRONGLY,
  STATE_ANSWERED_CORRECTLY
};

enum PowerState {
  GREEN,
  YELLOW,
  RED
};

bool watchDogTimerOn();

void updatePowerState();

float readVoltage();

void updatePowerIndicator();

void updateProgramState();

void setProgramState(ProgramState newState);

void resetChallenge();

int calculateAnswer();

void showAnswer();

void displayError();

void activateSolenoid();

void blinkTillTheEnd();
