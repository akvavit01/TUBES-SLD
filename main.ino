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
#define BAUD_RATE 115200 // Value of baud rate
#define VACUUMING_TIME 30000 // 5 minutes
#define KILLING_TIME 900000 // 15 minutes
#define DELAY_TIME 1000 // 1 second
// Remember, input from every sensor is measured in bits and returned as milivolts
// To be edited with values gained from calibration
#define MAX_PRESSURE 3000
#define MIN_KILLING_PRESSURE 1500 
#define MIN_KILLING_TEMPERATURE 1500
#define SAFE_MAX_PRESSURE 500
#define SAFE_MAX_TEMPERATURE 500

// Pin mapping
#define PRESSURE_SENSOR_INPUT_PIN PA0 // Pin for reading pressure sensor data
#define TEMPERATURE_SENSOR_INPUT_PIN PA1 // Pin for reading temperature sensor data
#define STEAM_INPUT_VALVE PA2 // Pin for controlling input steam
#define STEAM_OUTPUT_VALVE PA3 // Pin for controlling output steam
#define VACUUM_LID PA4 // Pin for controlling autoclave machine lock
#define ON_BUTTON PA5 // Pin for reading ON button input
#define FAILSAFE_BUTTON PA6 // Pin for reading failsafe button

// Class definitions
class AnalogSensor
{
    private :
        byte pin{};

    public :
        AnalogSensor(byte pin)
        {
            this->pin = pin;
            init();
        }

        void init()
        {
            pinMode(pin, INPUT);
        }

        unsigned long read() // returns voltage in milivolts
        {
            const unsigned long ADCValue {analogRead(pin) };
            const unsigned long refVoltage {3300};
            const unsigned long ADCResolution {4096};

            const unsigned long result {ADCValue * refVoltage / ADCResolution};

            return result;
        }
};

class Button // It is assumed that every button used is the pulldown variant
{
    private :
        byte pin{};
        byte state{};
        boolean previousReading{};

    public :
        Button(byte pin)
        {
            this->pin = pin;
            previousReading = LOW;
            init();
        }

        void init()
        {
            pinMode(pin, INPUT);
        }

        boolean isPressed()
        {
            boolean newReading{digitalRead(pin) };
            
            if (previousReading==LOW && newReading==HIGH)
            {
                previousReading = HIGH;
                return true;
            }
            else
            {
                previousReading = newReading;
                return false;
            }
        }
};

class Actuator
{
    private :
        byte pin{};

    public :
        Actuator(byte pin)
        {
            this->pin = pin;
            init();
        }

        void init()
        {
            pinMode(pin, OUTPUT);
            off();
        }

        void on()
        {
            digitalWrite(pin, HIGH);
        }

        void off()
        {
            digitalWrite(pin, LOW);
        }
};

// Actuator and sensors initialization
// Initialize pressure sensor
AnalogSensor pressureSensor(PRESSURE_SENSOR_INPUT_PIN);

// Initialize temperature sensor
AnalogSensor temperatureSensor(TEMPERATURE_SENSOR_INPUT_PIN);

// on() member function assumed to open both valves
// Initialize steam input valve
Actuator steamInput(STEAM_INPUT_VALVE);

// Initialize steam output valve
Actuator steamOutput(STEAM_OUTPUT_VALVE);

// Initialize vacuum lid
// on() member function assumed to seal the vacuum lid
Actuator vacuumLid(VACUUM_LID);   

// Initialize ON button
Button ONButton(ON_BUTTON);

// Initialize failsafe button
Button failsafeButton(FAILSAFE_BUTTON);

void setup()
{
	// Initialize serial monitor
    Serial.begin(BAUD_RATE); 
}

// Global variables declaration
enum MachineState
{
    IDLING,
    STERILIZING
};
MachineState machineState{IDLING};

enum VacuumState
{
    NOT_VACUUM,
    VACUUM
};
VacuumState vacuumState{NOT_VACUUM};

enum SterilizingStage
{
    STEAM_INTAKING,
    CONSTANT_TEMPERATURE_AND_PRESSURE,
    STEAM_EXHAUSTING
};
SterilizingStage sterilizingStage{};

unsigned long timeElapsed{0};

// Function prototype declaration
boolean isVacuum();
boolean killingFinished();

void loop()
{
    // If failsafe button is pressed, machine state changes to idling
    // Must be the highest priority!!!
    if (failsafeButton.isPressed() )
    {
        // Reporting condition via serial monitor
        Serial.println("Failsafe initiated!!!");
        Serial.println("Halting all processes!!!");

        // Changing machine state to idling
        machineState = IDLING;
    }

    else if (machineState == IDLING)
    {
        vacuumLid.off(); // Opening vacuum lid
        steamInput.off(); // Closing steam input valve
        steamOutput.on(); // Opening steam output valve

        // Sterilizing chamber is now not vacuum
        vacuumState = NOT_VACUUM;
    }

    else if (machineState == STERILIZING)
    {
        // Starting sterilizing process
        if (vacuumState == NOT_VACUUM)
        {
            // Reporting condition via serial monitor
            Serial.println("Now : vacuuming");

            // Removing air from sterilization chamber
            steamOutput.on(); // Opening output valve
            steamInput.on(); // Opening input valve

            // Checking if the chamber is already vacuum or not
            if (isVacuum() )
            {
                // Reporting condition via serial monitor
                Serial.println("Sterilization chamber is now vacuum");

                steamOutput.off(); // Closing output valve

                vacuumState = VACUUM; // Sterilizing chamber is now vacuum
                sterilizingStage = STEAM_INTAKING; // Changing sterilizing stage
            }
        }

        else if (vacuumState == VACUUM)
        {
            if (sterilizingStage == STEAM_INTAKING)
            {
                if (pressureSensor.read() >= MIN_KILLING_PRESSURE && temperatureSensor.read() >= MIN_KILLING_TEMPERATURE)
                {
                    // Reporting condition via serial monitor
                    Serial.println("Minimal temperature and pressure for killing reached!!!");
                    Serial.println("Now : killing time!!! 屮(☼Д☼)屮");

                    // Changing state
                    sterilizingStage = CONSTANT_TEMPERATURE_AND_PRESSURE;
                }
                else
                {
                    // Reporting condition via serial monitor
                    Serial.println("Maximum pressure exceeded!!!");
                    Serial.println("Now : reducing pressure");                    

                    // Letting steam out to prevent explosion
                    steamOutput.on(); // Letting steam out   
                }
            }

            else if (sterilizingStage == CONSTANT_TEMPERATURE_AND_PRESSURE)
            {
                // It's killing time!!! 屮(☼Д☼)屮
                // Reporting condition via serial monitor
                Serial.println("It's killing time!!! 屮(☼Д☼)屮");
                
                // Making sure temperature and pressure won't fall below minimal killing value
                if (pressureSensor.read() <= MIN_KILLING_PRESSURE || temperatureSensor.read() <= MIN_KILLING_TEMPERATURE)
                {
                    // Reporting condition via serial monitor
                    Serial.println("Temperature or pressure too low!!!");
                    Serial.println("Now : raising temperature and pressure");

                    steamInput.on(); // Opening steam input valve
                    steamOutput.off(); // Closing steam output valve
                }
                else if (pressureSensor.read() >= MAX_PRESSURE)
                {
                    // Reporting condition via serial monitor
                    Serial.println("Pressure too high!!!");
                    Serial.println("Now : lowering pressure");

                    steamInput.off(); // Closing steam input valve
                    steamOutput.on(); // Opening steam output valve
                }
                else
                {
                    steamInput.off(); // Closing steam input valve
                    steamOutput.off(); // Closing steam output valve
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
                // Reporting condition via serial monitor
                Serial.println("Now : steam exhausting");
                
                // Letting steam out
                steamOutput.on(); // Opening steam output valve
                steamInput.off(); // Closing steam input valve

                if (pressureSensor.read() <= SAFE_MAX_PRESSURE && temperatureSensor.read() <= SAFE_MAX_TEMPERATURE)
                {
                    // Reporting condition via serial monitor
                    Serial.println("Opening vacuum lid");

                    // Closing all valves
                    steamInput.off();
                    steamOutput.off();

                    // Opening vacuum lid
                    vacuumLid.off();

                    // Changing machine state
                    machineState = IDLING;
                }
            }
        }
    }

    // If ON button is pressed, machine state changes to sterilizing
    else if (ONButton.isPressed() )
    {
        machineState = STERILIZING;
        vacuumLid.on(); // Closing vacuum lid
    }

    // Making everything slower
    delay(DELAY_TIME);
}

// Functions definition
boolean isVacuum()
{
    // Checking if sterilization chamber is vacuum or not
    // Checking is done based on preset time
    if (timeElapsed >= VACUUMING_TIME)
    {
        timeElapsed = 0;
        return true;
    }
    else
    {
        timeElapsed += DELAY_TIME;
        return false;
    }
    
}

boolean killingFinished()
{
    // Checking if sterilization process is finished or not
    // Checking is done based on preset time
    if (timeElapsed >= KILLING_TIME)
    {
        timeElapsed = 0;
        return true;
    }
    else
    {
        timeElapsed += DELAY_TIME;
        return false;
    }
    
}
