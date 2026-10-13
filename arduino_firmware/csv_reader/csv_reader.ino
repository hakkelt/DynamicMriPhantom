#include <AccelStepper.h>
#include <Arduino.h>


// ============================================================
// PIN CONFIGURATION
// ============================================================

const int STEP_PIN = 6;
const int DIR_PIN  = 5;
const int BTN_PIN = 8;
const int MRI_PIN = 9;
const int ENABLE_PIN = -1;


// ============================================================
// STEPPER
// ============================================================

AccelStepper stepper(
    AccelStepper::DRIVER,
    STEP_PIN,
    DIR_PIN
);


// ============================================================
// TRAJECTORY STORAGE
// ============================================================

const uint16_t MAX_POINTS = 1024;

uint16_t trajectoryTime[MAX_POINTS];
int16_t trajectoryPosition[MAX_POINTS];

uint16_t numberOfPoints = 0;
uint16_t trajectoryPeriod = 0;


// ============================================================
// COMMUNICATION STATE
// ============================================================

bool receivingData = false;
bool trajectoryLoaded = false;

uint16_t expectedPoints = 0;
uint16_t receivedPoints = 0;


// ============================================================
// TRAJECTORY TIMING
// ============================================================

unsigned long cycleStartTime = 0;
bool buttonWasPressed = false;


// ============================================================
// SERIAL INPUT
// ============================================================

String inputLine;


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    inputLine.reserve(40);

    // DRV8825 ENABLE is active LOW
    pinMode(ENABLE_PIN, OUTPUT);
    pinMode(BTN_PIN, INPUT_PULLUP);
    pinMode(MRI_PIN, INPUT);
    digitalWrite(ENABLE_PIN, LOW);

    // Stepper configuration
    stepper.setMaxSpeed(5000.0);
    stepper.setAcceleration(4000.0);

    stepper.setCurrentPosition(0);
}


// ============================================================
// LOOP
// ============================================================

void loop()
{


    // --------------------------------------------------------
    // Process serial input
    // --------------------------------------------------------

    while (!trajectoryLoaded && Serial.available())
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

            // Basic protection against malformed input
            if (inputLine.length() > 60)
            {
                inputLine = "";
            }
        }
    }


    // --------------------------------------------------------
    // Execute trajectory
    // --------------------------------------------------------

    if (trajectoryLoaded && digitalRead(BTN_PIN) == LOW){
        buttonWasPressed = true;
    }


    if (trajectoryLoaded && buttonWasPressed)
    {
        runTrajectory();
    }
}



// ============================================================
// PROCESS COMMAND
// ============================================================

void processCommand(String command)
{
    // --------------------------------------------------------
    // HELLO
    // --------------------------------------------------------

    if (command == "HELLO")
    {

        Serial.println("READY");
        return;
    }


    // --------------------------------------------------------
    // SEND
    // --------------------------------------------------------

    if (command.startsWith("SEND "))
    {
        processSendHeader(command);
        return;
    }


    // --------------------------------------------------------
    // END
    // --------------------------------------------------------

    if (command == "END")
    {
        finishTransfer();
        return;
    }


    // --------------------------------------------------------
    // Trajectory data
    // --------------------------------------------------------

    if (receivingData)
    {
        processTrajectoryPoint(command);
        return;
    }


    // --------------------------------------------------------
    // Unknown command
    // --------------------------------------------------------

    Serial.print("ERROR UNKNOWN: ");
    Serial.println(command);
}


// ============================================================
// PROCESS SEND HEADER
// ============================================================
//
// Expected:
//
// SEND 128 6283
//
// meaning:
//
// 128 points
// 6283 ms period
//
// ============================================================

void processSendHeader(String command)
{
    int firstSpace = command.indexOf(' ');
    int secondSpace = command.indexOf(
        ' ',
        firstSpace + 1
    );


    if (firstSpace < 0 || secondSpace < 0)
    {
        Serial.println("ERROR HEADER");
        return;
    }


    String pointsString =
        command.substring(
            firstSpace + 1,
            secondSpace
        );


    String periodString =
        command.substring(
            secondSpace + 1
        );


    long points =
        pointsString.toInt();

    long period =
        periodString.toInt();


    // --------------------------------------------------------
    // Validate
    // --------------------------------------------------------

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


    // --------------------------------------------------------
    // Prepare for transfer
    // --------------------------------------------------------

    expectedPoints = (uint16_t)points;
    receivedPoints = 0;

    trajectoryPeriod = (uint16_t)period;

    receivingData = true;
    trajectoryLoaded = false;


    Serial.println("OK HEADER");
}


// ============================================================
// PROCESS TRAJECTORY POINT
// ============================================================
//
// Expected:
//
// 1234,567
//
// time_ms,position
//
// ============================================================

void processTrajectoryPoint(String line)
{
    int comma = line.indexOf(',');


    if (comma < 0)
    {
        Serial.println("ERROR FORMAT");
        return;
    }


    String timeString =
        line.substring(
            0,
            comma
        );


    String positionString =
        line.substring(
            comma + 1
        );


    long timeValue =
        timeString.toInt();


    long positionValue =
        positionString.toInt();


    // --------------------------------------------------------
    // Validate
    // --------------------------------------------------------

    if (timeValue < 0 || timeValue > 65535)
    {
        Serial.println("ERROR TIME");
        return;
    }


    if (positionValue < -32768 ||
        positionValue > 32767)
    {
        Serial.println("ERROR POSITION");
        return;
    }


    if (receivedPoints >= MAX_POINTS)
    {
        Serial.println("ERROR MEMORY");
        return;
    }


    // --------------------------------------------------------
    // Store
    // --------------------------------------------------------

    trajectoryTime[receivedPoints] =
        (uint16_t)timeValue;


    trajectoryPosition[receivedPoints] =
        (int16_t)positionValue;


    // --------------------------------------------------------
    // Acknowledge
    // --------------------------------------------------------

    Serial.print("OK ");
    Serial.println(receivedPoints);


    receivedPoints++;
}


// ============================================================
// FINISH TRANSFER
// ============================================================

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


    // --------------------------------------------------------
    // Transfer successful
    // --------------------------------------------------------

    numberOfPoints = receivedPoints;

    receivingData = false;
    trajectoryLoaded = true;


    // Start trajectory at first point
    stepper.moveTo(
        trajectoryPosition[0]
    );

    stepper.run();


    cycleStartTime = millis();

    if(trajectoryLoaded){
        Serial.println("START");
    }
}


// ============================================================
// RUN TRAJECTORY
// ============================================================

void runTrajectory()
{
    if (numberOfPoints < 2)
    {
        return;
    }

    uint16_t index = 0;

    // --------------------------------------------------------
    // Time within current cycle
    // --------------------------------------------------------

    while(1){
        unsigned long elapsed =
            millis() - cycleStartTime;


        // --------------------------------------------------------
        // Restart cycle
        // --------------------------------------------------------

        if (elapsed >= trajectoryPeriod)
        {
            cycleStartTime += trajectoryPeriod;

            elapsed =
                millis() - cycleStartTime;
            index = 0;
        }


        // --------------------------------------------------------
        // Find surrounding trajectory points
        // --------------------------------------------------------



        for (uint16_t i = index;
            i < numberOfPoints - 1;
            i++)
        {
            if (elapsed >= trajectoryTime[i] &&
                elapsed < trajectoryTime[i + 1])
            {
                index = i;
                break;
            }
        }

        stepper.moveTo(trajectoryPosition[index]);


        // --------------------------------------------------------
        // Get points for interpolation
        // --------------------------------------------------------

        /*uint16_t t0 =
            trajectoryTime[index];

        uint16_t t1 =
            trajectoryTime[index + 1];


        int16_t p0 =
            trajectoryPosition[index];

        int16_t p1 =
            trajectoryPosition[index + 1];


        // --------------------------------------------------------
        // Linear interpolation
        // --------------------------------------------------------

        float alpha;


        if (t1 == t0)
        {
            alpha = 0.0;
        }
        else
        {
            alpha =
                (float)(elapsed - t0) /
                (float)(t1 - t0);
        }


        float desiredPosition =
            p0 +
            alpha * (p1 - p0);


        long target =
            lround(desiredPosition);


        // --------------------------------------------------------
        // Command AccelStepper
        // --------------------------------------------------------

        //stepper.moveTo(int(millis()/10));
        //stepper.moveTo((int)(elapsed/10));
    */
        stepper.run();
    }
}