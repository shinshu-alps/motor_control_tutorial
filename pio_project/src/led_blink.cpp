/**
 * @file led_blink.cpp
 * @brief LEDの点滅
 */

#ifdef LED_BLINK

#include "mbed.h"

int main()
{
  DigitalOut led(LED1);

  while (true) {
    led = !led;
    ThisThread::sleep_for(500ms);
  }
}

#endif
