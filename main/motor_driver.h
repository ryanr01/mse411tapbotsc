#include <stdbool.h>  
#include <stdint.h>   

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

void motor_driver_init(void);
void motor_forward(void);
void motor_stop(void);
void motor_reverse(void);
void DCmotordrive(float target_distance_mm, bool spandir);

#endif // MOTOR_DRIVER_H
