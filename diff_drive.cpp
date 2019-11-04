#include <iostream>
#include <wiringPi.h>

using namespace std;

/*******************************************
 * NOTE
    pwmWrite(PWM_pin, value); 0 <= value < 1024
    digitalWrite(PIN, HIGH/LOW);
  >> gpio readall # show pin layout

 ******************************************/

// PWM pins on RPI4
constexpr int PWM_LEFT  = 1;    // PIN 12
constexpr int PWR_RIGHT = 23;   // PIN 33

constexpr int LOGIC_IN_1 = 24;  // pin 35
constexpr int LOGIC_IN_2 = 25;  // pin 37
constexpr int LOGIC_IN_3 = 27;  // pin 36
constexpr int LOGIC_IN_4 = 28;  // pin 38

int PWM_VALUE = 100;  // 0~1024

void stop()
{
    digitalWrite(LOGIC_IN_1, LOW);
    digitalWrite(LOGIC_IN_2, LOW);
    digitalWrite(LOGIC_IN_3, LOW);
    digitalWrite(LOGIC_IN_4, LOW);
}

void turnLeft(int duration)
{
    digitalWrite(LOGIC_IN_1, LOW);
    digitalWrite(LOGIC_IN_2, HIGH);
    digitalWrite(LOGIC_IN_3, HIGH);
    digitalWrite(LOGIC_IN_4, LOW);
    delay(duration);
    // stop();
}

void turnRight(int duration)
{
    digitalWrite(LOGIC_IN_1, HIGH);
    digitalWrite(LOGIC_IN_2, LOW);
    digitalWrite(LOGIC_IN_3, LOW);
    digitalWrite(LOGIC_IN_4, HIGH);
    delay(duration);
    // stop();
}

void forward(int duration)
{
    digitalWrite(LOGIC_IN_1, HIGH);
    digitalWrite(LOGIC_IN_2, LOW);
    digitalWrite(LOGIC_IN_3, HIGH);
    digitalWrite(LOGIC_IN_4, LOW);
    delay(duration);
    // stop();
}

void reverse(int duration)
{
    digitalWrite(LOGIC_IN_1, LOW);
    digitalWrite(LOGIC_IN_2, HIGH);
    digitalWrite(LOGIC_IN_3, LOW);
    digitalWrite(LOGIC_IN_4, HIGH);
    delay(duration);
    // stop();
}

int main (void)
{
    int intensity ;

    if (wiringPiSetup () == -1)
        exit (1) ;

    pinMode(PWM_LEFT, PWM_OUTPUT);  // set PIN 12 as PWM output
    pinMode(PWR_RIGHT, PWM_OUTPUT); // set PIN 33 as PWM output

    pinMode(LOGIC_IN_1, OUTPUT); // set PIN 35 as output
    pinMode(LOGIC_IN_2, OUTPUT); // set PIN 37 as output
    pinMode(LOGIC_IN_3, OUTPUT); // set PIN 36 as output
    pinMode(LOGIC_IN_4, OUTPUT); // set PIN 38 as output

    pwmWrite(PWM_LEFT, PWM_VALUE);     /* provide PWM0 value for duty cycle */
    pwmWrite(PWR_RIGHT, PWM_VALUE);    /* provide PWM1 value for duty cycle */

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
            cin >> PWM_VALUE;
            pwmWrite(PWM_LEFT, PWM_VALUE);
            pwmWrite(PWR_RIGHT, PWM_VALUE);
        }
        else
        {
            delay(100);
        }
    }
}
