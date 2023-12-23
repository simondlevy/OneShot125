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

static int8_t direction;

static bool gotInput;

void serialEvent(void)
{
    while (Serial.available()) {
        Serial.read();
    }

    gotInput = true;
}

void setup() 
{
    Serial.begin(115200);

    while (!gotInput) {
        Serial.println("Hit Enter to begin ...");
        delay(1000);
    }

    gotInput = false;

    esc.arm(); 

    Serial.println("Hit enter to stop ... ");

    pulseWidth = 125;

    direction = +1;
}

void loop() 
{
    const auto loopStartUsec = micros();      

    esc.set(pulseWidth); 

    loopDelay(2000, loopStartUsec); 

    if (gotInput) {
        pulseWidth = 125;
    }

    else {
        static uint32_t prev;
        auto msec = millis();
        if (msec - prev > 100) {
            pulseWidth += direction;
            prev = msec;
            if (pulseWidth == 250) {
                direction = -1;
            }
            if (pulseWidth == 125) {
                direction = +1;
            }
        }
    }
}
