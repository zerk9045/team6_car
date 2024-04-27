#include "IRSensor.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "../config/pin_config.h"

IRSensor::IRSensor() {
    gpio_init(IR_SENSOR_PIN);
    gpio_set_dir(IR_SENSOR_PIN, GPIO_IN);
    gpio_set_irq_enabled_with_callback(IR_SENSOR_PIN, GPIO_IRQ_EDGE_FALL, true, &IRSensor::do_interrupt);

    timer_init();
    timer_set_callback();
    timer_start();
}

int IRSensor::getSpeed() {
    speed = (speed * 0.1) - 1;
    return speed;
}

void IRSensor::do_interrupt(uint gpio, uint32_t events, uintptr_t data) {
    IRSensor *instance = reinterpret_cast<IRSensor *>(data);
    instance->sensor_interrupts++;
}

void IRSensor::timer_action() {
    speed = sensor_interrupts;
    printf("Counts per second = %d\n", speed);
    sensor_interrupts = 0;
}

void IRSensor::timer_init() {
    // Allocate a hardware timer
    timer = hardware_timer_claim_unused();
    if (timer == -1) {
        // Handle error: no unused timers
    }

    // Configure the timer
    hardware_timer_set_source(timer, clk_sys);
    hardware_timer_set_count_direction(timer, hardware_timer_count_direction_up);
}

void IRSensor::timer_set_callback() {
    // Set the callback function
    hardware_timer_set_callback(timer, &IRSensor::timer_action, this);
}

void IRSensor::timer_start() {
    // Start the timer to repeat every 1 second (1,000,000 microseconds)
    hardware_timer_set_interval_us(timer, 1000000);
    hardware_timer_set_enabled(timer, true);
}