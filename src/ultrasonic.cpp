#include "ultrasonic.h"

uint32_t get_distance(int echo_pin, int trig_pin)
{
    digitalWrite(trig_pin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trig_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_pin, LOW);

    return pulseIn(echo_pin, HIGH, 30000UL) / 58;
}

#define ROUND_ROBIN_MS 100

uint32_t us_f = 0;
uint32_t us_fl = 0;
uint32_t us_fr = 0;

void ultrasonic_task(void *pvParameters)
{
    for (;;)
    {
        us_f = get_distance(ECHO_F, TRIG_F);

        delay(ROUND_ROBIN_MS);

        us_fl = get_distance(ECHO_FL, TRIG_FL);
        delay(ROUND_ROBIN_MS);

        us_fr = get_distance(ECHO_FR, TRIG_FR);
        delay(ROUND_ROBIN_MS);
    }

    vTaskDelete(NULL);
}