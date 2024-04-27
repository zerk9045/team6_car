//
// Created by Raphael Bret-Mounet on 4/27/24.
//

#ifndef TEAM6_CAR_IRSENSOR_H
#define TEAM6_CAR_IRSENSOR_H


class IRSensor {
    public:
        // Constructor
        IRSensor();

        // Method to get the speed
        int getSpeed();

        // Method to handle interrupts
        static void do_interrupt(uint gpio, uint32_t events, uintptr_t data);
    private:


        // Method to handle timer actions
        void timer_action();

        // Variable to store the number of interrupts
        int sensor_interrupts;

        // Variable to store the speed;
        int speed;

};


#endif //TEAM6_CAR_IRSENSOR_H
