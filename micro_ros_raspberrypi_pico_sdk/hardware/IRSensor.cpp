#include "IRSensor.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "../config/pin_config.h"

IRSensor::IRSensor() {
    // Initialize the sensor pin as input
    gpio_init(IR_SENSOR_PIN);
    gpio_set_dir(IR_SENSOR_PIN, GPIO_IN);

    // Set up the interrupt on falling edge
    gpio_set_irq_enabled_with_callback(IR_SENSOR_PIN, GPIO_IRQ_EDGE_FALL, true, &IRSensor::do_interrupt);

    // Initialize the timer
    timer_init();
    timer_set_callback(&timer, &IRSensor::timer_action);

    // Start the timer
    timer_start(&timer, 1000, true); // 1000 ms = 1 second
}

int IRSensor::getSpeed() {
    // Convert speed to m/s
    speed = (speed * 0.1) - 1;  // 0.1 is the conversion factor from counts per second to m/s
    return speed;
}

void IRSensor::do_interrupt() {
    // Increment the interrupt count
    sensor_interrupts++;
}

void IRSensor::timer_action() {
    // This method is called every second by the timer

    // Calculate the speed by getting the number of interrupts per second
    speed = sensor_interrupts;

    // Print the speed to the console
    printf("Counts per second = %d\n", speed);

    // Reset the interrupt count
    sensor_interrupts = 0;
}