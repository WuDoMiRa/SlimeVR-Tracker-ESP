/// Extra functions.
#include "globals.h"

namespace SlimeVR {
    void led_on(){ digitalWrite(LED_PIN, LED__ON); } // turn ON LED
    void led_off(){ digitalWrite(LED_PIN, LED__OFF); } // turn ON LED
    void led_flash(int times=2, int delaytime=1) {
        for (int i = 0; i < times; i++) {
            led_on();
            delay(delaytime);
            led_off();
            delay(delaytime);
        }
    }
    void led_blink(int delaytime=1) { // makes the led blink
        led_flash(1, delaytime);
    } 

}