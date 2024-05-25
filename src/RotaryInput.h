#ifndef ROTARY_INPUT_H
#define ROTARY_INPUT_H

#include <FR_RotaryEncoder.h>
#include <Arduino.h>

class RotaryInput {
  public:
    static RotaryInput& init(byte pinA, byte pinB, byte pinButton);

    void reset();
    uint8_t getDigitsEntered();
    int getAnswer();

  private:
    static RotaryInput* INSTANCE;

    static void CHANGED_CALLBACK();
    static void PRESSED_CALLBACK();
  
    void rotaryChangedCallback();
    void rotaryPressedCallback();

    RotaryInput(byte pinA, byte pinB, byte pinButton);
    
    byte pinA;
    byte pinButton;

    RotaryEncoder rotaryEncoder;

    volatile uint8_t digitsEntered = 0;
    volatile unsigned int partialAnswer = 0;
    volatile uint8_t lastDigitValue = 0;
    volatile RotaryEncoder::SwitchState lastSwitchState = RotaryEncoder::SwitchState::SW_OFF;
};

#endif