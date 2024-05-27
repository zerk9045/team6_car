//
// Created by Raphael Bret-Mounet on 4/27/24.
//
#include "pico/stdlib.h"
#ifndef TEAM6_CAR_IRSENSOR_H
#define TEAM6_CAR_IRSENSOR_H

#define INTERRUPT_TIME_MS 70
#define INTERRUPT_TIME_US  (INTERRUPT_TIME_MS*1000)
class IRSensor {
    public:
        // Constructor
        IRSensor();

        // Method to get the speed
        int getCountsPerTimer();

        // Method to handle interrupts
        static void do_interrupt(uint gpio, uint32_t events);
        static void resetSensorInterrupts();
        void resetCounts();
    private:


        // Method to handle timer actions
        static bool prev_state;
        // Variable to store the number of interrupts
        static int sensor_interrupts;

        // Variable to store the speed;
        int speed;
        static int counts_per_timer;
};


#endif //TEAM6_CAR_IRSENSOR_H
