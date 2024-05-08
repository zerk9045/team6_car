#include "IRSensor.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "../config/pin_config.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"

IRSensor irSensor;

IRSensor::IRSensor() {
    

    gpio_init(IR_SENSOR_PIN);
    gpio_set_dir(IR_SENSOR_PIN, GPIO_IN);
    gpio_set_irq_enabled_with_callback(IR_SENSOR_PIN, GPIO_IRQ_EDGE_FALL, true, &IRSensor::interrupt_callback);
}


int IRSensor::getSpeed() {
    repeating_timer_callback_t rt;
    add_repeating_timer_ms(1000, rt, NULL, NULL);
    
    speed = sensor_interrupts;
    sensor_interrupts = 0;
    return speed;
}

void IRSensor::do_interrupt(uint gpio, uint32_t events) {
    
    sensor_interrupts = sensor_interrupts + 1;
}

void IRSensor::interrupt_callback(unsigned int gpio, long unsigned int events) {
    irSensor.do_interrupt(gpio, events);
}