#include <AccelStepper.h>
#include <Arduino.h>

// ============================================================
// PIN CONFIGURATION
// ============================================================
const int STEP_PIN   = 6;
const int DIR_PIN    = 5;
const int BTN_PIN    = 8;
const int MRI_PIN    = 9;
const int ENABLE_PIN = -1; // Set pin number if DRV8825 enable pin is connected

// ============================================================
// STEPPER INSTANCE
// ============================================================
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// ============================================================
// TRAJECTORY STORAGE
// ============================================================
const uint16_t MAX_POINTS = 1024;

uint16_t trajectoryTime[MAX_POINTS];
int16_t trajectoryPosition[MAX_POINTS];

uint16_t numberOfPoints   = 0;
uint16_t trajectoryPeriod = 0;

// ============================================================
// SYSTEM STATES & CONTROL FLAGS
// ============================================================
enum SystemMode {
    MODE_IDLE,
    MODE_TRAJECTORY_WAIT_TRIGGER,
    MODE_TRAJECTORY_RUNNING,
    MODE_MANUAL_MOVING
};

SystemMode currentMode  = MODE_IDLE;
bool waitMri            = false;
bool receivingData      = false;
bool trajectoryLoaded   = false;

uint16_t expectedPoints = 0;
uint16_t receivedPoints = 0;
unsigned long cycleStartTime = 0;

String inputLine;

// Function declarations
void processCommand(String command);
void processSendHeader(String command);
void processTrajectoryPoint(String line);
void finishTransfer();

// ============================================================
// SETUP
// ============================================================
void setup()
{
    Serial.begin(115200);
    inputLine.reserve(64);

    if (ENABLE_PIN >= 0) {
        pinMode(ENABLE_PIN, OUTPUT);
        digitalWrite(ENABLE_PIN, LOW); // DRV8825 enable is active LOW
    }

    pinMode(BTN_PIN, INPUT_PULLUP);
    pinMode(MRI_PIN, INPUT);

    stepper.setMaxSpeed(10000.0);
    stepper.setAcceleration(10000.0);
    stepper.setCurrentPosition(0);
}

// ============================================================
// MAIN LOOP (NON-BLOCKING)
// ============================================================
void loop()
{
    // --------------------------------------------------------
    // 1. Non-blocking Serial Command Processing
    // --------------------------------------------------------
    while (Serial.available() > 0)
    {
        char c = Serial.read();

        if (c == '\n')
        {
            inputLine.trim();
            if (inputLine.length() > 0)
            {
                processCommand(inputLine);
            }
            inputLine = "";
        }
        else if (c != '\r')
        {
            inputLine += c;
            if (inputLine.length() > 80)
            {
                inputLine = ""; // Discard malformed long buffers
            }
        }
    }

    // --------------------------------------------------------
    // 2. Trajectory Execution & Trigger State Machine
    // --------------------------------------------------------
    if (currentMode == MODE_TRAJECTORY_WAIT_TRIGGER)
    {
        if (digitalRead(MRI_PIN) == HIGH || digitalRead(BTN_PIN) == LOW)
        {
            currentMode = MODE_TRAJECTORY_RUNNING;
            cycleStartTime = millis();
            Serial.println("TRIGGERED");
        }
    }

    if (currentMode == MODE_TRAJECTORY_RUNNING)
    {
        if (numberOfPoints >= 2)
        {
            unsigned long elapsed = millis() - cycleStartTime;

            // Restart continuous trajectory cycle
            if (elapsed >= trajectoryPeriod)
            {
                cycleStartTime += trajectoryPeriod;
                elapsed = millis() - cycleStartTime;
            }

            // Find keyframe interval
            uint16_t index = 0;
            for (uint16_t i = 0; i < numberOfPoints - 1; i++)
            {
                if (elapsed >= trajectoryTime[i] && elapsed < trajectoryTime[i + 1])
                {
                    index = i;
                    break;
                }
            }

            // Linear interpolation between trajectory keyframes
            uint16_t t0 = trajectoryTime[index];
            uint16_t t1 = trajectoryTime[index + 1];
            int16_t p0  = trajectoryPosition[index];
            int16_t p1  = trajectoryPosition[index + 1];

            float alpha = 0.0f;
            if (t1 > t0)
            {
                alpha = (float)(elapsed - t0) / (float)(t1 - t0);
            }

            long targetPos = lround((float)p0 + alpha * (float)(p1 - p0));
            stepper.moveTo(targetPos);
        }
    }
    else if (currentMode == MODE_MANUAL_MOVING)
    {
        if (stepper.distanceToGo() == 0)
        {
            currentMode = MODE_IDLE;
        }
    }

    // --------------------------------------------------------
    // 3. Step Motor Motor Driver Generator
    // --------------------------------------------------------
    stepper.run();
}

// ============================================================
// PROCESS COMMANDS
// ============================================================
void processCommand(String command)
{
    if (command == "HELLO")
    {
        Serial.println("READY");
        return;
    }

    if (command.startsWith("WAITMRI "))
    {
        int val = command.substring(8).toInt();
        waitMri = (val == 1);
        Serial.print("OK WAITMRI ");
        Serial.println(waitMri ? 1 : 0);

        if (currentMode == MODE_TRAJECTORY_WAIT_TRIGGER && !waitMri)
        {
            currentMode = MODE_TRAJECTORY_RUNNING;
            cycleStartTime = millis();
            Serial.println("START");
        }
        return;
    }

    if (command.startsWith("SEND "))
    {
        processSendHeader(command);
        return;
    }

    if (command == "END")
    {
        finishTransfer();
        return;
    }

    if (command == "RESET")
    {
        currentMode = MODE_MANUAL_MOVING;
        stepper.moveTo(0);
        Serial.println("OK RESET");
        return;
    }

    if (command == "SETZERO")
    {
        stepper.setCurrentPosition(0);
        Serial.println("OK SETZERO");
        return;
    }

    if (command.startsWith("MOVE "))
    {
        long steps = command.substring(5).toInt();
        currentMode = MODE_MANUAL_MOVING;
        stepper.move(steps);
        Serial.print("OK MOVE ");
        Serial.println(steps);
        return;
    }

    if (command == "STOP")
    {
        currentMode = MODE_IDLE;
        stepper.stop();
        Serial.println("OK STOP");
        return;
    }

    if (receivingData)
    {
        processTrajectoryPoint(command);
        return;
    }

    Serial.print("ERROR UNKNOWN: ");
    Serial.println(command);
}

void processSendHeader(String command)
{
    int firstSpace  = command.indexOf(' ');
    int secondSpace = command.indexOf(' ', firstSpace + 1);

    if (firstSpace < 0 || secondSpace < 0)
    {
        Serial.println("ERROR HEADER");
        return;
    }

    long points = command.substring(firstSpace + 1, secondSpace).toInt();
    long period = command.substring(secondSpace + 1).toInt();

    if (points <= 0 || points > MAX_POINTS)
    {
        Serial.println("ERROR POINTS");
        return;
    }

    if (period <= 0 || period > 65535)
    {
        Serial.println("ERROR PERIOD");
        return;
    }

    expectedPoints   = (uint16_t)points;
    receivedPoints   = 0;
    trajectoryPeriod = (uint16_t)period;

    receivingData    = true;
    trajectoryLoaded = false;
    currentMode      = MODE_IDLE;

    Serial.println("OK HEADER");
}

void processTrajectoryPoint(String line)
{
    int comma = line.indexOf(',');
    if (comma < 0)
    {
        Serial.println("ERROR FORMAT");
        return;
    }

    long timeValue     = line.substring(0, comma).toInt();
    long positionValue = line.substring(comma + 1).toInt();

    if (timeValue < 0 || timeValue > 65535 || positionValue < -32768 || positionValue > 32767 || receivedPoints >= MAX_POINTS)
    {
        Serial.println("ERROR DATA");
        return;
    }

    trajectoryTime[receivedPoints]     = (uint16_t)timeValue;
    trajectoryPosition[receivedPoints] = (int16_t)positionValue;

    Serial.print("OK ");
    Serial.println(receivedPoints);

    receivedPoints++;
}

void finishTransfer()
{
    if (!receivingData)
    {
        Serial.println("ERROR NO_TRANSFER");
        return;
    }

    if (receivedPoints != expectedPoints)
    {
        Serial.print("ERROR COUNT ");
        Serial.print(receivedPoints);
        Serial.print("/");
        Serial.println(expectedPoints);
        receivingData = false;
        return;
    }

    numberOfPoints   = receivedPoints;
    receivingData    = false;
    trajectoryLoaded = true;

    if (waitMri)
    {
        currentMode = MODE_TRAJECTORY_WAIT_TRIGGER;
        Serial.println("WAITING_TRIGGER");
    }
    else
    {
        currentMode = MODE_TRAJECTORY_RUNNING;
        cycleStartTime = millis();
        Serial.println("START");
    }
}