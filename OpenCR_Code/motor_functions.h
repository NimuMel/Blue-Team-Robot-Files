#ifndef MOTOR_FUNCTIONS_H
#define MOTOR_FUNCTIONS_H

#include "macros.h"
#include"ros_functions.h"

// External variables used in the functions
extern int32_t tilt_home;
extern int32_t sort_pos;
extern int32_t tilt_pos;

// Function declarations
void setupDynWheels();
void autoHome(uint8_t motor_id);
void autoHomeScoop();
void sortInit();
void sortLeft();
void sortRight();
void tiltUp();
void tiltDown();
void scoopUp();
void scoopDown();
void autoHomeDump();
void dumpUp();
void dumpDown();


#endif // MOTOR_FUNCTIONS_H