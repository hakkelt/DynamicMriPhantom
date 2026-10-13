#include <Arduino.h>

// Define pin connections & motor's steps per revolution
const int dirPin = 5;
const int stepPin = 6;
const int buttonPin = 8; //5
const int mriPin = 9; //7
const int stepsPerRevolution = 200; //200
const int waitTime = 2000;

int counter = 0;

boolean rotating = false;

void setup() {
  // Declare pins as Outputs
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  
  // Declare button pin as Input with internal pull-up
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(mriPin, INPUT);
}

void loop() {
  // Check if button is pressed (LOW when pressed due to pull-up)
  if (digitalRead(buttonPin) == LOW or digitalRead(mriPin) == HIGH) { //digitalRead(buttonPin) == LOW or 
    rotating = true;
    counter = 0;
  }
  
  //rotating = true;

  // Keep rotating if enabled
  if (rotating && counter < 10) {
    // Set motor direction clockwise
    digitalWrite(dirPin, HIGH);

    // Spin motor slowly
    for (int x = 0; x < stepsPerRevolution; x++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(waitTime);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(waitTime);
    }

    // Set motor direction counterclockwise
    digitalWrite(dirPin, LOW);

    // Spin motor quickly
    for (int x = 0; x < stepsPerRevolution; x++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(waitTime);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(waitTime);
    }

  counter += 1;

  }
}