#include "IRSensor.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "../config/pin_config.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "pico/time.h"
#include "stdio.h"
// Declare a global instance of IRSensor

// Initialize static member
int IRSensor::sensor_interrupts = 0;
int IRSensor::counts_per_timer = 0;
// Declare a hardware timer
absolute_time_t timer;
// Declare a timer pool
alarm_pool_t *timer_pool;
// Interrupt handler for the timer

int64_t timer_interrupt(alarm_id_t id, void *user_data) {
    IRSensor::resetSensorInterrupts();
    return 1000000;
}

IRSensor::IRSensor() {
    gpio_init(IR_SENSOR_PIN);
    gpio_set_dir(IR_SENSOR_PIN, GPIO_IN);
    gpio_set_irq_enabled_with_callback(IR_SENSOR_PIN, GPIO_IRQ_EDGE_FALL, true, IRSensor::do_interrupt);
    // Initialize the hardware timer
    //timer = make_timeout_time_ms(1000);
    add_alarm_in_us(1000000, timer_interrupt, NULL, true);
}

void IRSensor::resetSensorInterrupts() {
    counts_per_timer = sensor_interrupts;
    sensor_interrupts = 0;
    //printf("Interrupts: %d\n", counts_per_timer);
}

int IRSensor::getCountsPerTimer() {
    return counts_per_timer;
}

void IRSensor::do_interrupt(uint gpio, uint32_t events) {
    sensor_interrupts++;
}

