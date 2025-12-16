#include <Arduino.h>

// Define pin connections & motor's steps per revolution
const int dirPin = 2;
const int stepPin = 3;
const int buttonPin = 5;
const int stepsPerRevolution = 200;

boolean rotating = false;

void setup() {
  // Declare pins as Outputs
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  
  // Declare button pin as Input with internal pull-up
  pinMode(buttonPin, INPUT_PULLUP);
}

void loop() {
  // Check if button is pressed (LOW when pressed due to pull-up)
  if (digitalRead(buttonPin) == LOW) {
    rotating = true;
  }
  
  // Keep rotating if enabled
  if (rotating) {
    // Set motor direction clockwise
    digitalWrite(dirPin, HIGH);

    // Spin motor slowly
    for (int x = 0; x < stepsPerRevolution; x++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(2000);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(2000);
    }

    // Set motor direction counterclockwise
    digitalWrite(dirPin, LOW);

    // Spin motor quickly
    for (int x = 0; x < stepsPerRevolution; x++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(1000);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(1000);
    }
  }
}
