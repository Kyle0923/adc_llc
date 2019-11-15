#include <iostream>
#include <pigpiod_if2.h>
#include "pca9685_if.hpp"

using namespace std;

// PWM pins on RPI4

constexpr unsigned LOGIC_IN_1 = 16;  // pin 36
constexpr unsigned LOGIC_IN_2 = 20;  // pin 38
constexpr unsigned LOGIC_IN_3 = 19;  // pin 35
constexpr unsigned LOGIC_IN_4 = 26;  // pin 37

uint32_t PWM_MAX = 4096;  // max
double VOLT_MAX = 12; // volt

static int piHandle = -1;

void stop()
{
    gpio_write(piHandle, LOGIC_IN_1, 0);
    gpio_write(piHandle, LOGIC_IN_2, 0);
    gpio_write(piHandle, LOGIC_IN_3, 0);
    gpio_write(piHandle, LOGIC_IN_4, 0);
}

void turnLeft(float duration)
{
    gpio_write(piHandle, LOGIC_IN_1, 0);
    gpio_write(piHandle, LOGIC_IN_2, 1);
    gpio_write(piHandle, LOGIC_IN_3, 1);
    gpio_write(piHandle, LOGIC_IN_4, 0);
    time_sleep(duration);
}

void turnRight(float duration)
{
    gpio_write(piHandle, LOGIC_IN_1, 1);
    gpio_write(piHandle, LOGIC_IN_2, 0);
    gpio_write(piHandle, LOGIC_IN_3, 0);
    gpio_write(piHandle, LOGIC_IN_4, 1);
    time_sleep(duration);
}

void forward(float duration)
{
    gpio_write(piHandle, LOGIC_IN_1, 1);
    gpio_write(piHandle, LOGIC_IN_2, 0);
    gpio_write(piHandle, LOGIC_IN_3, 1);
    gpio_write(piHandle, LOGIC_IN_4, 0);
    time_sleep(duration);
}

void reverse(float duration)
{
    gpio_write(piHandle, LOGIC_IN_1, 0);
    gpio_write(piHandle, LOGIC_IN_2, 1);
    gpio_write(piHandle, LOGIC_IN_3, 0);
    gpio_write(piHandle, LOGIC_IN_4, 1);
    time_sleep(duration);
}

void exitFunction() {
    pigpio_stop(piHandle);
}

int main (void)
{
    piHandle = pigpio_start(nullptr, nullptr);
    if (piHandle < 0)
    {
        return -1;
    }
    std::atexit(exitFunction);

    Pca9685IF pca9685(piHandle);

    int setPinStatus = 0;

    setPinStatus |= set_mode(piHandle, LOGIC_IN_1, PI_OUTPUT);  // set PIN 35 as output
    setPinStatus |= set_mode(piHandle, LOGIC_IN_2, PI_OUTPUT);  // set PIN 37 as output
    setPinStatus |= set_mode(piHandle, LOGIC_IN_3, PI_OUTPUT);  // set PIN 36 as output
    setPinStatus |= set_mode(piHandle, LOGIC_IN_4, PI_OUTPUT);  // set PIN 38 as output

    if (setPinStatus != 0)
    {
        return -2;
    }

    float duration = 1;

    std::string input = "";
    while (1)
    {
        stop();
        cout << "Waiting for input W-A-S-D" << endl;
        cin >> input;
        if (input == "a")
        {
            cout << "turnLeft" << endl;
            turnLeft(duration);
        }
        else if (input == "d")
        {
            cout << "turnRight" << endl;
            turnRight(duration);
        }
        else if (input == "s")
        {
            cout << "reverse" << endl;
            reverse(duration);
        }
        else if (input == "w")
        {
            cout << "forward" << endl;
            forward(duration);
        }
        else if (input == "q")
        {
            stop();
        }
        else if (input == "exit")
        {
            stop();
            break;
        }
        else if (input == "config")
        {
            cin >> input;
            double value;
            cin >> value;
            if (input == "L")
            {
                // value == dutyCycle
                value = value / 100;
                pca9685.setLeftPwm(static_cast<uint8_t>(value * PWM_MAX));
                cout << "LEFT PWM value: " << value * PWM_MAX << ", Voltage: " << value * VOLT_MAX << " V" << endl;
            }
            else if (input == "R")
            {
                // value == dutyCycle
                value = value / 100;
                pca9685.setRightPwm(static_cast<uint8_t>(value * PWM_MAX));
                cout << "ROGHT PWM value: " << value * PWM_MAX << ", Voltage(V): " << value * VOLT_MAX << " V" << endl;
            }
            else if (input == "D")
            {
                duration = value;
                cout << "Duration changed to: " << duration << " sec" << endl;
            }
        }
        else if (input == "help" || input == "-h" || input == "--help")
        {
            cout << "============================== Usage ==============================" << endl;
            cout << "W-A-S-D:\t\tDirection" << endl;
            cout << "q:\t\t\tstop (both pwm to 0)" << endl;
            cout << "config <flag> [num]:\tchange config corresponding to flag" << endl;
            cout << "\t\t\tflag:" << endl;
            cout << "\t\t\t     L: left wheel pwm duty cycle (percentage)" << endl;
            cout << "\t\t\t     R: right wheel pwm duty cycle (percentage)" << endl;
            cout << "\t\t\t     D: duration of movement in seconds" << endl;
            cout << "reset:\t\t\treset base_link frame coordinate" << endl;
            cout << "exit:\t\t\texit program" << endl;
            continue;
        }
    }
    return 0;
}
