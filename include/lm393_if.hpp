#ifndef LM393_IF_HPP
#define LM393_IF_HPP

void lm393Callback(int pi, unsigned user_gpio, unsigned level, uint32_t tick);

class Lm393IF
{
    public:
        Lm393IF(int aPiHandle, unsigned aPin);
        Lm393IF() = delete;
        ~Lm393IF() = default;
        uint32_t getReading();
        uint32_t incrementReading();

    private:
        const unsigned mPin;
        unsigned mLevel;
        uint32_t mReading;
        int callbackID;

};

#endif //IM393_IF_HPP