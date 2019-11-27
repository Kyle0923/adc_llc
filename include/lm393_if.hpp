#ifndef LM393_IF_HPP
#define LM393_IF_HPP

void lm393Callback(int pi, unsigned user_gpio, unsigned level, uint32_t tick);

// one full rotation is 40 changes in encoder value

class Lm393IF
{
    public:
        Lm393IF(int aPiHandle, unsigned aPin);
        Lm393IF() = delete;
        ~Lm393IF();
        uint32_t incrementReading();
        double getAngularDisplacement(); // rad

    private:
        static constexpr double READING_OF_FULL_REVOLUTION = 40.0;
        const int mPiHandle;
        const unsigned mPin;
        uint32_t mReading;
        int mCallbackID;

};

#endif //IM393_IF_HPP