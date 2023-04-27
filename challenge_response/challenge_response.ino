#include <TM1637Display.h>

#define CLK 2
#define DIO 3
#define LED_B 5
#define LED_G 6
#define LED_R 9
#define VOLTAGE_DIVIDER_FACTOR ((100.0 + 10.0) / 10.0)

TM1637Display display = TM1637Display(CLK, DIO);
int i = 0;

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600); // TODO: Remove after debugging

  display.clear();  
  display.setBrightness(7);

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  analogReference(INTERNAL);
  pinMode(A0, INPUT);
  i = 0;
}

void loop() {
  display.showNumberDec(i);
  indicateVoltage();
  delay(1000);
  i++;
}

void indicateVoltage() {
  // read the input on analog pin 0:
  int sensorValue = analogRead(A0);
  // Convert the analog reading (which goes from 0 - 1023) to the reference voltage (0 - 1.1V) and compensate for the voltage divider.
  float voltage = sensorValue * (1.1 / 1023.0) * VOLTAGE_DIVIDER_FACTOR;

  // print out the value you read:
  Serial.println(voltage); // TODO: Remove after debugging

  if (voltage < 3.5) {
    // Make LED red
    analogWrite(LED_R, 1);
    analogWrite(LED_G, 0);
    analogWrite(LED_B, 0);
  } else if (voltage < 3.8) {
    // Make LED yellow
    analogWrite(LED_R, 1);
    analogWrite(LED_G, 1);
    analogWrite(LED_B, 0);
  } else {
    // Make LED green
    analogWrite(LED_R, 0);
    analogWrite(LED_G, 1);
    analogWrite(LED_B, 0);
  }
}
