#include "Servo.h"
#include "pico/stdlib.h"
#include "../config/pin_config.h" // Include the pin configuration header
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#define STRAIGHT_ANGLE_PWM 1500000
#define MAX_ANGLE_PWM 1625000
#define MIN_ANGLE_PWM 1375000

Servo::Servo(){
    gpio_init(SERVO_PWM);

    //Setup up PWM t
    gpio_set_function(SERVO_PWM, GPIO_FUNC_PWM);
    pwm_set_gpio_level(SERVO_PWM, 0);
    uint slice_num = pwm_gpio_to_slice_num(SERVO_PWM);

    // Get clock speed and compute divider for 50 hz
    uint32_t clk = clock_get_hz(clk_sys);
    uint32_t div = clk / (20000 * 100);

    // Check div is in range
    if ( div < 1 ){
        div = 1;
    }
    if ( div > 255 ){
        div = 255;
    }

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, (float)div);

    // Set wrap so the period is 20 ms
    pwm_config_set_wrap(&config, 20000);

    // Load the configuration
    pwm_init(slice_num, &config, false);

    pwm_set_enabled(slice_num, true);
}

Servo::~Servo() {
}

void set_pwm_pin(int pin, uint freq, uint duty_c) { // duty_c between 0..10000
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(pin);
    pwm_config config = pwm_get_default_config();
    float div = (float)clock_get_hz(clk_sys) / (freq * 10000);
    pwm_config_set_clkdiv(&config, div);
    pwm_config_set_wrap(&config, 10000);
    pwm_init(slice_num, &config, true); // start the pwm running according to the config
    pwm_set_gpio_level(pin, duty_c); //connect the pin to the pwm engine and set the on/off level.
}

void Servo::setAngle(int anglePWM) {
    // Ensure pwm is within the specified range
    if (anglePWM > MAX_ANGLE_PWM) {
        anglePWM = MAX_ANGLE_PWM;
    } else if (anglePWM < MIN_ANGLE_PWM) {
        anglePWM = MIN_ANGLE_PWM;
    }

    // Calculate duty cycle
    float dutyCycle = (float)(anglePWM - MIN_ANGLE_PWM) / (MAX_ANGLE_PWM - MIN_ANGLE_PWM);
    uint16_t pwmValue = (uint16_t)(dutyCycle * 65535); // 65535 is the maximum PWM value (2^16 - 1)

    // Set PWM duty cycle for servo control pin
    set_pwm_pin(SERVO_PWM, 100, pwmValue);

    currAnglePWM = anglePWM;
}



int Servo::getAngle() {
    // convert the pwm signal to an angle
//    int rangePWM = MAX_ANGLE_PWM - MIN_ANGLE_PWM;
//    float scale = 2.0 / rangePWM; // Adjust the scale to map to the range -1 to 1
//    int normalizedAngle = ((currAnglePWM - MIN_ANGLE_PWM) * scale) - 1; // Subtract 1 to shift the range to -1 to 1

    return currAnglePWM;
}