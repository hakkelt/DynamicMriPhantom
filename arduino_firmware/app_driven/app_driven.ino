#include <ArduinoJson.h>
#include <math.h>
#include <AccelStepper.h>

// --- Hardware Setup ---
// Define pins for your driver (Adjust these to your actual wiring!)
#define STEP_PIN 6
#define DIR_PIN  5
#define BTN_PIN 8
#define MRI_PIN 7

// Initialize the stepper (DRIVER mode uses Step and Direction pins)
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

enum State { STOPPED, DWELLING, MOVING, RESETTING, ARMED };
State currentState = STOPPED;

// Motion Parameters
float amplitude = 0;
unsigned long moveTime = 0;
unsigned long dwellTime = 0;
float gamma_val = 1.0;
int microstep;
long lastTargetPos = -1;
bool mri_signal;
int cycles;
int current_cycle = 0;

// Timing Variables
unsigned long phaseStartTime = 0;

void setup() {
  Serial.begin(115200);
  
  // Set stepper limits (Adjust based on your motor's torque/speed)
  stepper.setMaxSpeed(6000);
  stepper.setAcceleration(6000);

  //pinMode(MRI_PIN, INPUT); //INPUT for START (1/2)
}

void loop() {
  // 1. Process Serial Commands
  if (Serial.available() > 0) {
    String payload = Serial.readStringUntil('\n');
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, payload);

    if (!error) {
      String cmd = doc["cmd"];
      
      if (cmd == "stop") {
        currentState = STOPPED;
        stepper.stop(); // Decelerate to a stop
        Serial.println("{\"status\":\"ack\", \"cmd\":\"stop\"}");
      } 
      else if (cmd == "reset") {
        currentState = RESETTING;
        stepper.moveTo(0);
        Serial.println("{\"status\":\"ack\", \"cmd\":\"reset\"}");
      } 
      else if (cmd == "start") {
        amplitude = doc["params"]["amplitude"];
        moveTime  = doc["params"]["move_ms"];
        dwellTime = doc["params"]["dwell_ms"];
        gamma_val = doc["params"]["gamma"];
        microstep = doc["params"]["microstep"];
        mri_signal = doc["params"]["mri_signal"];
        cycles    = doc["params"]["cycles"];

        current_cycle = 1;
        
        if(mri_signal){
          currentState = ARMED;
        }
        else{
          currentState = MOVING;
          phaseStartTime = millis();
        }

        /*Serial.print("Cmd:");
        Serial.print(cmd);
        Serial.print(", Amplitude:");
        Serial.print(amplitude);
        Serial.print(", moveTime:");
        Serial.print(moveTime);
        Serial.print(", dwellTime:");
        Serial.print(dwellTime);
        Serial.print(", gamma_val:");
        Serial.print(gamma_val);
        Serial.print(", microstep:");
        Serial.print(microstep);
        Serial.print(", MRI:");
        Serial.print(mri_signal);*/

        Serial.println(" | {\"status\":\"ack\", \"cmd\":\"start\"}");
        //Serial.end();
      }
    }
  }


  //if(currentState == ARMED) {
  //  if (digitalRead(MRI_PIN) == HIGH) { //HIGH for START INPUT (2/2)
  //    currentState = MOVING;
  //    phaseStartTime = millis();
  //  }
  //}

  if (currentState == MOVING || currentState == DWELLING) {
    unsigned long elapsed = millis() - phaseStartTime;

    if (elapsed <= moveTime) {
      // Using 2000.0 forces floating-point math (e.g., 0.5 * steps)
      float progress = (float)elapsed / moveTime;
      long targetPos = (1.0 - cos(TWO_PI * pow((1-progress),gamma_val))) * float(amplitude) * float(microstep) / 1.8;


      if (targetPos != lastTargetPos) {
        stepper.moveTo(targetPos);
        lastTargetPos = targetPos; // Only update when it actually changes
      }

    } else if(elapsed <= moveTime + dwellTime){ // Start dwelling and move back to 0 in the middle of dwelling
      currentState = DWELLING;

      if(elapsed > moveTime + 0.5 * dwellTime){
        stepper.moveTo(0);
      }

    } else if(current_cycle < cycles){
      currentState = MOVING;
      phaseStartTime = millis();
      current_cycle++;

    }else{
      currentState = STOPPED;
      //Serial.begin(115200);
    }
  }


  // CRITICAL: This must run every loop to actually step the motor!
  stepper.run();
}