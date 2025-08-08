#ifndef ROTARY_INPUT_H
#define ROTARY_INPUT_H

#include <FR_RotaryEncoder.h>
#include <Arduino.h>

/**
 * Represents an answer being entered with a rotary encoder.
 * 
 * When the rotary's switch is pushed, the rotary's current value (0-9) is multiplied by 10 and the number of entered 
 * digits is incremented by one.
 */
class AnswerInput {
  public:
    /**
     * Initializes the answer input with the given hardware pins.
     */
    static AnswerInput& init(byte pinA, byte pinB, byte pinButton);

    /**
     * Resets the answer input to a 'no answer given' state.
     */
    void reset();

    /** 
     * Returns the number of digits that were entered by pressing the rotary switch.
     */
    uint8_t getDigitsEntered();

    /**
     * Returns the current (possibly incomplete) answer.
     * 
     * A negative value indicates that no answer is being entered yet.
     */
    int getAnswer();

    /**
     * Returns whether the answer input has changed and resets it.
     */
    bool getAndResetUpdate();

    /** 
     * Returns true if all digits are entered.
     */
    bool isFinalAnswer();

  private:
    static AnswerInput* INSTANCE;

    static void CHANGED_CALLBACK();
    static void PRESSED_CALLBACK();
  
    void rotaryChangedCallback();
    void rotaryPressedCallback();

    AnswerInput(byte pinA, byte pinB, byte pinButton);
    
    byte pinA;
    byte pinButton;

    RotaryEncoder rotaryEncoder;

    volatile uint8_t digitsEntered = 0;
    volatile unsigned int partialAnswer = 0;
    volatile RotaryEncoder::SwitchState lastSwitchState = RotaryEncoder::SwitchState::SW_OFF;
    volatile bool isChanged = false;
};

#endif