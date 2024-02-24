#ifndef ROTARY_INPUT_H
#define ROTARY_INPUT_H

#include <FR_RotaryEncoder.h>
#include <Arduino.h>

class RotaryInput {
  public:
    static RotaryInput& init(byte pinA, byte pinB, byte pinButton, void (*inputChangedCallback)());

    void reset();
    uint8_t getDigitsEntered();
    unsigned int getAnswer();

  private:
    static RotaryInput* INSTANCE;

    static void globalChangeCallback();
    static void globalPressedCallback();
  
    void rotaryChangeCallback();
    void rotaryPressedCallback();

    RotaryInput(byte pinA, byte pinB, byte pinButton, void (*inputUpdatedCallback)());
    
    byte pinA;
    byte pinButton;
    void (*inputUpdatedCallback)();

    RotaryEncoder rotaryEncoder;

    volatile uint8_t digitsEntered = 0;
    volatile unsigned int answer = 0;
    volatile uint8_t lastDigitValue = 0;
    volatile RotaryEncoder::SwitchState lastSwitchState = RotaryEncoder::SwitchState::SW_OFF;

};

#endif