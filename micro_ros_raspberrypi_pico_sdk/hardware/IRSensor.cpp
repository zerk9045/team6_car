#include "IRSensor.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "../config/pin_config.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"

IRSensor::IRSensor() {
    sensor_interrupts = 0;
    gpio_init(IR_SENSOR_PIN);
    gpio_set_dir(IR_SENSOR_PIN, GPIO_IN);
    gpio_set_irq_enabled_with_callback(IR_SENSOR_PIN, GPIO_IRQ_EDGE_FALL, true, &IRSensor::do_interrupt);
}


int IRSensor::getSpeed() {
//    repeating_timer_t timer;
//
//
//    auto callback = [](repeating_timer_t *t) {
//        // Do not increment sensor_interrupts here
//        return true; // return true to keep the timer repeating
//    };
//
//
//    add_repeating_timer_ms(1000, callback, NULL, &timer);

    return sensor_interrupts;
}

void IRSensor::do_interrupt(uint gpio, uint32_t events) {
    
    sensor_interrupts = sensor_interrupts + 1;
}

