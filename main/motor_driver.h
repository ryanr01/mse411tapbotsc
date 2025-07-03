#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

void motor_driver_init(void);
void motor_forward(void);
void motor_stop(void);
void motor_reverse(void);
void DCmotordrive(float target_distance_mm, int direction);

#endif // MOTOR_DRIVER_H