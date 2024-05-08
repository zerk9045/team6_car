//
// Created by Raphael Bret-Mounet on 4/27/24.
//
#include "pico/stdlib.h"
#ifndef TEAM6_CAR_IRSENSOR_H
#define TEAM6_CAR_IRSENSOR_H


class IRSensor {
    public:
        // Constructor
        IRSensor();

        // Method to get the speed
        int getSpeed();

        // Method to handle interrupts
        static void do_interrupt(uint gpio, uint32_t events);
        void resetSensorInterrupts();
        
    private:


        // Method to handle timer actions

        // Variable to store the number of interrupts
        static int sensor_interrupts;

        // Variable to store the speed;
        int speed;
};


#endif //TEAM6_CAR_IRSENSOR_H
