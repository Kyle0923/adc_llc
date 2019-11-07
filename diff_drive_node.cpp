#include <iostream>
#include <pigpiod_if2.h>

using namespace std;

/*******************************************
 * NOTE
    pwmWrite(PWM_pin, value); 0 <= value < 1024
    gpio_write(piHandle, PIN, 1/0);
  >> gpio readall # show pin layout

 ******************************************/

// PWM pins on RPI4
constexpr unsigned PWM_LEFT  = 12;   // PIN 32
constexpr unsigned PWR_RIGHT = 13;   // PIN 33

constexpr unsigned LOGIC_IN_1 = 16;  // pin 36
constexpr unsigned LOGIC_IN_2 = 20;  // pin 38
constexpr unsigned LOGIC_IN_3 = 19;  // pin 35
constexpr unsigned LOGIC_IN_4 = 26;  // pin 37

constexpr unsigned PWM_FREQ = 10000000;    //10M Hz

uint32_t PWM_VALUE = 100000;  // 60000~1000000, 1M

static int piHandle = -1;

void stop()
{
    gpio_write(piHandle, LOGIC_IN_1, 0);
    gpio_write(piHandle, LOGIC_IN_2, 0);
    gpio_write(piHandle, LOGIC_IN_3, 0);
    gpio_write(piHandle, LOGIC_IN_4, 0);
}

void turnLeft(uint32_t duration)
{
    gpio_write(piHandle, LOGIC_IN_1, 0);
    gpio_write(piHandle, LOGIC_IN_2, 1);
    gpio_write(piHandle, LOGIC_IN_3, 1);
    gpio_write(piHandle, LOGIC_IN_4, 0);
    time_sleep(duration / 1000.0);
    // stop();
}

void turnRight(uint32_t duration)
{
    gpio_write(piHandle, LOGIC_IN_1, 1);
    gpio_write(piHandle, LOGIC_IN_2, 0);
    gpio_write(piHandle, LOGIC_IN_3, 0);
    gpio_write(piHandle, LOGIC_IN_4, 1);
    time_sleep(duration / 1000.0);
    // stop();
}

void forward(uint32_t duration)
{
    gpio_write(piHandle, LOGIC_IN_1, 1);
    gpio_write(piHandle, LOGIC_IN_2, 0);
    gpio_write(piHandle, LOGIC_IN_3, 1);
    gpio_write(piHandle, LOGIC_IN_4, 0);
    time_sleep(duration / 1000.0);
    // stop();
}

void reverse(uint32_t duration)
{
    gpio_write(piHandle, LOGIC_IN_1, 0);
    gpio_write(piHandle, LOGIC_IN_2, 1);
    gpio_write(piHandle, LOGIC_IN_3, 0);
    gpio_write(piHandle, LOGIC_IN_4, 1);
    time_sleep(duration / 1000.0);
    // stop();
}

void exitFunction() {
    pigpio_stop(piHandle);
}

int main (void)
{
    piHandle = pigpio_start(nullptr, nullptr);
    if (piHandle < 0)
    {
        return 1;
    }
    std::atexit(exitFunction);

    int setPinStatus = 0;
    setPinStatus |= hardware_PWM(piHandle, PWM_LEFT, PWM_FREQ, PWM_VALUE);  // set PIN 12 as PWM output
    setPinStatus |= hardware_PWM(piHandle, PWR_RIGHT, PWM_FREQ, PWM_VALUE); // set PIN 13 as PWM output

    setPinStatus |= set_mode(piHandle, LOGIC_IN_1, PI_OUTPUT);  // set PIN 35 as output
    setPinStatus |= set_mode(piHandle, LOGIC_IN_2, PI_OUTPUT);  // set PIN 37 as output
    setPinStatus |= set_mode(piHandle, LOGIC_IN_3, PI_OUTPUT);  // set PIN 36 as output
    setPinStatus |= set_mode(piHandle, LOGIC_IN_4, PI_OUTPUT);  // set PIN 38 as output

    if (setPinStatus != 0)
    {
        return 2;
    }

    // pwmWrite(PWM_LEFT, PWM_VALUE);     /* provide PWM0 value for duty cycle */
    // pwmWrite(PWR_RIGHT, PWM_VALUE);    /* provide PWM1 value for duty cycle */

    std::string input = "";
    while (1)
    {
        cout << "Waiting for input W-A-S-D" << endl;
        cin >> input;
        if (input == "a")
        {
            cout << "turnLeft" << endl;
            turnLeft(500);
        }
        else if (input == "d")
        {
            cout << "turnRight" << endl;
            turnRight(500);
        }
        else if (input == "s")
        {
            cout << "reverse" << endl;
            reverse(500);
        }
        else if (input == "w")
        {
            cout << "forward" << endl;
            forward(500);
        }
        else if (input == "q")
        {
            stop();
        }
        else if (input == "config")
        {
            cin >> input;
            cin >> PWM_VALUE;
            if (input == "L")
            {
                hardware_PWM(piHandle, PWM_LEFT, PWM_FREQ, PWM_VALUE);
            }
            else if (input == "R")
            {
                hardware_PWM(piHandle, PWR_RIGHT, PWM_FREQ, PWM_VALUE);
            }
        }
        else
        {
            time_sleep(100);
        }
    }
    return 0;
}
