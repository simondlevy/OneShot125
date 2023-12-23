#include <oneshot125.hpp>

static const uint8_t ESC_PIN = 0;

static auto esc = OneShot125(ESC_PIN);

static void loopDelay(const uint32_t freq, const uint32_t loopStartUsec) 
{

    auto invFreq = 1.0/freq*1000000.0;

    auto time = micros();

    while (invFreq > (time - loopStartUsec)) {
        time = micros();
    }
}

static uint8_t pulseWidth;

void setup() 
{
    Serial.begin(115200);

    esc.arm(); 

    while (true) {

        Serial.println("Hit enter to begin ...");

        if (Serial.available()) {
            break;
        }

        delay(1000);
    }

    pulseWidth = 125;
}

void loop() 
{

    const auto loopStartUsec = micros();      

    esc.set(pulseWidth); 

    loopDelay(2000, loopStartUsec); 

    static uint32_t prev;
    auto msec = millis();
    if (msec - prev > 100) {
        pulseWidth += 1;
        prev = msec;
        if (pulseWidth == 250) {
            pulseWidth = 130;
        }
    }
}
