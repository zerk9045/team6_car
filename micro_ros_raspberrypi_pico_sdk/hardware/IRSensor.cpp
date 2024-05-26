#include "IRSensor.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "../config/pin_config.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "pico/time.h"
#include "stdio.h"
// int64_t minimize(something){


// // Declare a global instance of IRSensor

// // Initialize static member
// int IRSensor::sensor_interrupts = 0;
// int IRSensor::counts_per_timer = 0;
// // Declare a hardware timer
// absolute_time_t irTimer;
// // Declare a timer pool
// alarm_pool_t *timer_pool;
// // Interrupt handler for the timer
// absolute_time_t last_interrupt_time;

// int64_t timer_interrupt(alarm_id_t id, void *user_data) {
//     IRSensor::resetSensorInterrupts();
//     return 200000;
// }

// IRSensor::IRSensor() {
//     gpio_init(IR_SENSOR_PIN);
//     gpio_set_dir(IR_SENSOR_PIN, GPIO_IN);
//     gpio_set_irq_enabled_with_callback(IR_SENSOR_PIN, GPIO_IRQ_EDGE_RISE, true, IRSensor::do_interrupt);
//     // Initialize the hardware timer
//     irTimer = make_timeout_time_ms(200);
//     add_alarm_in_us(200000, timer_interrupt, NULL, true);
// }

// // Function for reseting the number of sensor interrupts
// void IRSensor::resetSensorInterrupts() {
//     sensor_interrupts = 0;
//     //printf("Interrupts: %d\n", counts_per_timer);
// }

// // Function for getting the current number of counts in a certain interval
// int IRSensor::getCountsPerTimer() {
//     return counts_per_timer;
// }

// // Function for reseting the number of counts
// void IRSensor::resetCounts(){
// 	counts_per_timer = 0;
// }

// void IRSensor::do_interrupt(uint gpio, uint32_t events) {
//     // Get the current time
//     absolute_time_t now = get_absolute_time();

//     // Calculate the time difference since the last interrupt
//     int64_t diff_us = absolute_time_diff_us(last_interrupt_time, now);

//     // If the time difference is less than the debounce period (e.g., 1000 microseconds),
//     // then this interrupt is likely due to noise, so we ignore it
//     // Will have to adjust this time.
//     if (diff_us > 800) {
//         // Otherwise, this is a valid interrupt, so we update the last interrupt time
//         // and increment the interrupt counter
//         last_interrupt_time = now;
//         //If IR_SENSOR PIN is high then increment the counter
//         if (gpio_get(IR_SENSOR_PIN)) {
//            sensor_interrupts++;
//             counts_per_timer = sensor_interrupts;
//         }
//     }
// }
// }

// Initialize static member
int IRSensor::sensor_interrupts = 0;
int IRSensor::counts_per_timer = 0;     // number of revolutions read by the IR sensor

// Declare a hardware timer
absolute_time_t irTimer;

// Declare a timer pool
alarm_pool_t *timer_pool;

// Interrupt handler for the timer
absolute_time_t last_interrupt_time;   

int64_t timer_interrupt(alarm_id_t id, void *user_data) {
    IRSensor::resetSensorInterrupts();
    return 200000;
}

IRSensor::IRSensor() {
    gpio_init(IR_SENSOR_PIN);
    gpio_set_dir(IR_SENSOR_PIN, GPIO_IN);
    gpio_set_irq_enabled_with_callback(IR_SENSOR_PIN, GPIO_IRQ_EDGE_FALL, true, IRSensor::do_interrupt);
    // Initialize the hardware timer
    irTimer = make_timeout_time_ms(200);
    add_alarm_in_us(200000, timer_interrupt, NULL, true);
}

// Function for reseting the number of sensor interrupts
void IRSensor::resetSensorInterrupts() {
    sensor_interrupts = 0;
}

// Function for getting the current number of counts in a certain interval
int IRSensor::getCountsPerTimer() {
    return counts_per_timer / 2;
}

// Function for reseting the number of counts
void IRSensor::resetCounts(){
	counts_per_timer = 0;
}

// readEncoder function
void IRSensor::do_interrupt(uint gpio, uint32_t events) {

    // Debouncing logic
    // Get the current time
    absolute_time_t now = get_absolute_time();

    // Calculate the time difference since the last interrupt
   int64_t diff_us = absolute_time_diff_us(last_interrupt_time, now);

    // Check if the current rising edge is the actual rising edge or if it is part of the bouncing
    // If the time difference is greater than 8 ms, it is a valid rising edge
    if (diff_us > 8000 && !gpio_get(IR_SENSOR_PIN)){

        // Set the timestamp of this interrupt as the previous interrupt time for the next iteration
        last_interrupt_time = now;

        // Increment the number of revolutions
        counts_per_timer = counts_per_timer + 1;
    }
}
