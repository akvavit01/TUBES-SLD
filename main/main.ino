/**
 * Program for automated autoclave machine
 * Using gravity-displacement for vacuuming
 * Written by : 
 * Leonardi
 * Github : github.com/akvavit01
 * e-mail : leochen123987@gmail.com
 * 
 * Designed with STM32F108C6 board in mind
 * Use STM32 Cores library (https://github.com/stm32duino/Arduino_Core_STM32)
 * or Arduino_STM32 (https://github.com/rogerclarkmelbourne/Arduino_STM32)
 * 
 * Why STM32F108C6? It's cheaper and way faster than UNO or Nano!
 * 
 * P.S. : The logic level is 3.3V
 */

// Header inclusion
#include <Arduino.h>

// Constants declaration
#define BAUD_RATE       115200 // Value of baud rate
#define VACUUMING_TIME  30000 // 5 minutes
#define KILLING_TIME    900000 // 15 minutes
// Remember, input from every sensor is measured in bits and returned as milivolts
// To be edited with values gained from calibration
#define MAX_PRESSURE            3000
#define MIN_KILLING_PRESSURE    1500 
#define MIN_KILLING_TEMPERATURE 1500
#define SAFE_MAX_PRESSURE       500
#define SAFE_MAX_TEMPERATURE    500
#define REFERENCE_VOLTAGE       3300
#define ADC_RESOLUTION          4096
// For setting button type
#define PULLDOWN    0
#define PULLUP      1
// For defining solenoid valve type
#define NORMALLY_CLOSED  0
#define NORMALLY_OPEN    1

// Pin mapping
#define PRESSURE_SENSOR_INPUT_PIN       PA0 // Pin for reading pressure sensor data
#define TEMPERATURE_SENSOR_INPUT_PIN    PA1 // Pin for reading temperature sensor data
#define STEAM_INPUT_VALVE               PA2 // Pin for controlling input steam
#define STEAM_OUTPUT_VALVE              PA3 // Pin for controlling output steam
#define VACUUM_LID                      PA4 // Pin for controlling autoclave machine lock
#define ON_BUTTON                       PA5 // Pin for reading ON button input
#define FAILSAFE_BUTTON                 PA6 // Pin for reading failsafe button

// Enums for declaring machine states
enum MachineState
{
    IDLING,
    STERILIZING
};

enum VacuumState
{
    NOT_VACUUM,
    VACUUM
};

enum SterilizingStage
{
    STEAM_INTAKING,
    CONSTANT_TEMPERATURE_AND_PRESSURE,
    STEAM_EXHAUSTING
};

// Class definitions
class AnalogSensor
{
    private :
        byte pin{};

    public :
        AnalogSensor(byte pin);
        void init();
        unsigned long read(); // returns voltage in milivolts
};

class Button
{
    private :
        byte pin{};
        byte state{};
        boolean previousReading{};
        boolean variant{};

    public :
        Button(byte pin, boolean variant);
        void init();
        boolean isPressed();
};

class SolenoidValve
{
    private :
        byte pin{};
        boolean variant{};

    public :
        SolenoidValve(byte pin, boolean variant);
        void init();
        void open();
        void close();
};

// Actuator and sensors initialization
// Initialize pressure sensor
AnalogSensor pressureSensor(PRESSURE_SENSOR_INPUT_PIN);
// Initialize temperature sensor
AnalogSensor temperatureSensor(TEMPERATURE_SENSOR_INPUT_PIN);
// Initialize steam input valve
SolenoidValve steamInput(STEAM_INPUT_VALVE, NORMALLY_CLOSED);
// Initialize steam output valve
SolenoidValve steamOutput(STEAM_OUTPUT_VALVE, NORMALLY_CLOSED);
// Initialize vacuum lid
SolenoidValve vacuumLid(VACUUM_LID, NORMALLY_OPEN);   
// Initialize ON button
Button ONButton(ON_BUTTON, PULLDOWN);
// Initialize failsafe button
Button failsafeButton(FAILSAFE_BUTTON, PULLDOWN);

// Function declaration
boolean isVacuum();
boolean killingFinished();

void setup()
{
	// Initialize serial monitor
    Serial.begin(BAUD_RATE); 
}

void loop()
{
    // Variables declaration
    static MachineState machineState{IDLING};
    static VacuumState vacuumState{NOT_VACUUM};
    static SterilizingStage sterilizingStage{STEAM_INTAKING};
    // Only printing message when messagePrinted = false
    // to prevent printing the same message over and over again
    static boolean messagePrinted{false};
    static boolean ONButtonPressed{false};

    // Checking states
    if (machineState == IDLING)
    {
        // Only printing once
        if (!messagePrinted)
        {
            Serial.println("Now : idling");
            messagePrinted = true;
        }

        vacuumLid.open(); // Opening vacuum lid
        steamInput.close(); // Closing steam input valve
        steamOutput.open(); // Opening steam output valve

        // Sterilizing chamber is now not vacuum
        vacuumState = NOT_VACUUM;
    }

    else if (machineState == STERILIZING)
    {
        // Starting sterilizing process
        if (vacuumState == NOT_VACUUM)
        {
            if (!messagePrinted)
            {
                Serial.println("Now : vacuuming");
                messagePrinted = true;
            }

            // Removing air from sterilization chamber
            steamOutput.open(); // Opening output valve
            steamInput.open(); // Opening input valve

            // Checking if the chamber is already vacuum or not
            if (isVacuum() )
            {
                Serial.println("Sterilization chamber is now vacuum");
                Serial.println("Now : steam intaking");

                steamOutput.close(); // Closing output valve

                vacuumState = VACUUM; // Sterilizing chamber is now vacuum
                sterilizingStage = STEAM_INTAKING; // Changing sterilizing stage

                messagePrinted = false;
            }
        }

        else if (vacuumState == VACUUM)
        {
            if (sterilizingStage == STEAM_INTAKING)
            {
                if (pressureSensor.read() >= MIN_KILLING_PRESSURE && temperatureSensor.read() >= MIN_KILLING_TEMPERATURE)
                {
                    Serial.println("Minimal temperature and pressure for killing reached!!!");
                    Serial.println("Now : killing time!!! 屮(☼Д☼)屮");

                    // Changing state
                    sterilizingStage = CONSTANT_TEMPERATURE_AND_PRESSURE;

                    messagePrinted = false;
                }
                else if (pressureSensor.read() >= MAX_PRESSURE)
                {
                    if (!messagePrinted)
                    {
                        Serial.println("Maximum pressure exceeded!!!");
                        Serial.println("Now : reducing pressure");

                        messagePrinted = true;
                    }                    

                    // Letting steam out to prevent explosion
                    steamOutput.open(); // Letting steam out   
                }
                else
                {
                    messagePrinted = false;
                }
                
            }

            else if (sterilizingStage == CONSTANT_TEMPERATURE_AND_PRESSURE)
            {
                // It's killing time!!! 屮(☼Д☼)屮
                // Making sure temperature and pressure won't fall below minimal killing value
                if (pressureSensor.read() <= MIN_KILLING_PRESSURE || temperatureSensor.read() <= MIN_KILLING_TEMPERATURE)
                {
                    if (!messagePrinted)
                    {
                        Serial.println("Temperature or pressure too low!!!");
                        Serial.println("Now : raising temperature and pressure");

                        messagePrinted = true;
                    }

                    steamInput.open(); // Opening steam input valve
                    steamOutput.close(); // Closing steam output valve
                }
                else if (pressureSensor.read() >= MAX_PRESSURE)
                {
                    if (!messagePrinted)
                    {
                        Serial.println("Pressure too high!!!");
                        Serial.println("Now : lowering pressure");

                        messagePrinted = true;
                    }

                    steamInput.close(); // Closing steam input valve
                    steamOutput.open(); // Opening steam output valve
                }
                else
                {
                    steamInput.close(); // Closing steam input valve
                    steamOutput.close(); // Closing steam output valve

                    messagePrinted = false;
                }

                if (killingFinished() )
                {
                    // Reporting condition via serial monitor
                    Serial.println("Killing finished!!!");
                    Serial.println("Now : letting steam out");

                    // Changing state
                    sterilizingStage = STEAM_EXHAUSTING;
                }
            }

            else if (sterilizingStage == STEAM_EXHAUSTING)
            {
                // Letting steam out
                steamOutput.open(); // Opening steam output valve
                steamInput.close(); // Closing steam input valve

                if (pressureSensor.read() <= SAFE_MAX_PRESSURE && temperatureSensor.read() <= SAFE_MAX_TEMPERATURE)
                {
                    Serial.println("Sterilization process finished!!!");
                    Serial.println("Opening vacuum lid");

                    // Closing all valves
                    steamInput.close();
                    steamOutput.close();

                    // Opening vacuum lid
                    vacuumLid.open();

                    // Changing machine state
                    machineState = IDLING;

                    messagePrinted = false;
                    ONButtonPressed = false;
                }
            }
        }
    }

    // If failsafe button is pressed, machine state changes to idling
    if (failsafeButton.isPressed() )
    {
        // Reporting condition via serial monitor
        Serial.println("Failsafe initiated!!!");
        Serial.println("Halting all processes!!!");

        // Changing machine state to idling
        machineState = IDLING;
    }
    // If ON button is pressed, machine state changes to sterilizing
    else if (ONButton.isPressed() )
    {
        if (!ONButtonPressed)
        {
            Serial.println("ON button pressed!!! Starting sterilization process!!!");
            
            ONButtonPressed = true;
            messagePrinted = false;
        }

        machineState = STERILIZING;
        vacuumLid.close(); // Closing vacuum lid
    }
}

// Class method definitions
// Class AnalogSensor
AnalogSensor::AnalogSensor(byte pin)
{
    this->pin = pin;
    init();
}

void AnalogSensor::init()
{
    pinMode(pin, INPUT_ANALOG);
}

unsigned long AnalogSensor::read()
{
    const unsigned int ADCValue {analogRead(pin) };

    return (ulong)(ADCValue * REFERENCE_VOLTAGE / ADC_RESOLUTION);
}

// Class Button
Button::Button(byte pin, boolean variant)
{
    this->pin = pin;
    this->variant = variant;

    previousReading = LOW;

    init();
}

void Button::init()
{
    if (variant == PULLDOWN)
    {
        pinMode(pin, INPUT);
    }
    else
    {
        pinMode(pin, INPUT_PULLUP);
    }
}

boolean Button::isPressed()
{
    boolean newReading{digitalRead(pin) };
            
    // Checking button is pressed or not
    if (variant == PULLDOWN)
    {
        if (previousReading==LOW && newReading==HIGH)
        {
            previousReading = HIGH;
            return true;
        }
    }
    else
    {
        if (previousReading==HIGH && newReading==LOW)
        {
            previousReading = LOW;
            return true;
        }
    }

    // If button is not pressed
    previousReading = newReading;
    return false;
}

// Class SolenoidValve
SolenoidValve::SolenoidValve(byte pin, boolean variant)
{
    this->pin = pin;
    this->variant = variant;

    init();
}

void SolenoidValve::init()
{
    pinMode(pin, OUTPUT);
    close();
}

void SolenoidValve::open()
{
    if (variant == NORMALLY_CLOSED)
    {
        digitalWrite(pin, HIGH);
    }
    else
    {
        digitalWrite(pin, LOW);
    }
}

void SolenoidValve::close()
{
    if (variant == NORMALLY_OPEN)
    {
        digitalWrite(pin, HIGH);
    }
    else
    {
        digitalWrite(pin, LOW);
    }
}

// Functions definition
boolean isVacuum()
{
    static unsigned long timeElapsed{millis() };

    // Checking if sterilization chamber is vacuum or not
    // Checking is done based on preset time
    if (timeElapsed >= VACUUMING_TIME)
    {
        timeElapsed = 0;
        return true;
    }
    else
    {
        timeElapsed += (millis() - timeElapsed);
        return false;
    }
    
}

boolean killingFinished()
{
    static unsigned long timeElapsed{0};
    
    // Checking if sterilization process is finished or not
    // Checking is done based on preset time
    if (timeElapsed >= KILLING_TIME)
    {
        timeElapsed = 0;
        return true;
    }
    else
    {
        timeElapsed += (millis() - timeElapsed);
        return false;
    }
    
}
