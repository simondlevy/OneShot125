#include <oneshot125.hpp>

static const uint8_t PIN = 0;

static const uint32_t FREQ = 2000;

static auto esc = OneShot125(PIN);

static uint8_t pulseWidth;

static int8_t pulseIncrement;

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

    Serial.println("Hit Enter to stop ... ");

    pulseWidth = 125;

    pulseIncrement = +1;
}

void loop() 
{
    const auto loopStartUsec = micros();      

    esc.set(pulseWidth); 

    auto time = micros();

    while ((time - loopStartUsec) < 1.0/FREQ*1000000.0) {
        time = micros();
    }

    if (gotInput) {
        pulseWidth = 125;
    }

    else {
        static uint32_t prev;
        auto msec = millis();
        if (msec - prev > 100) {
            pulseWidth += pulseIncrement;
            prev = msec;
            if (pulseWidth == 250) {
                pulseIncrement = -1;
            }
            if (pulseWidth == 125) {
                pulseIncrement = +1;
            }
        }
    }
}
