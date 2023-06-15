/* Library to interface with 2.74 Motor Shield
** Uses low level HAL libraries to enable high speed PWM 
** Use as follows:
** - Create shield object and specify PWM period for the motors
** - Set the duty cycle and direction for each motor 
*/

#include "mbed.h"
#include "MotorShield.h"
#include "HardwareSetup.h"

//MotorA pins
DigitalOut enableA(PG_7);
DigitalOut disableA(PG_4);
//MotorB pins
DigitalOut enableB(PE_4);
DigitalOut disableB(PE_2);
//MotorC pins
DigitalOut enableC(PB_13);
DigitalOut disableC(PB_4);
//MotorD pins
DigitalOut enableD(PA_14);
DigitalOut disableD(PA_15);


MotorShield::MotorShield(int periodTicks) {
    periodTickVal = periodTicks; 
    init();
}
 
void MotorShield::init() {
    /** Initial config for the STM32H743 **/
    
    initHardware(periodTickVal);   // Setup PWM
    wait_us(100);

    //enable the motor A driver
    enableA.write(1);
    disableA.write(0);
    //enable the motor B driver
    enableB.write(1);
    disableB.write(0);
    //enable the motor C driver
    enableC.write(1);
    disableC.write(0);
    //enable the motor D driver
    enableD.write(1);
    disableD.write(0);
    wait_us(100);
}
 
void MotorShield::motorAWrite(float duty_cycle, int direction) {
    int tick = (int)(periodTickVal * duty_cycle); 

    if (direction){
            
            TIM15->CCR2 = tick;
            TIM15->CCR1 = tick;
    }
    else {
            TIM15->CCR2 = 0;
            TIM15->CCR1 = tick; 
    }
    
}

void MotorShield::motorBWrite(float duty_cycle, int direction) {
    int tick = (int)(periodTickVal * duty_cycle); 

    if (direction){
            TIM12->CCR2 = tick;
            TIM12->CCR1 = tick;
    }
    else {
            TIM12->CCR2 = 0;
            TIM12->CCR1 = tick;
    }

}

void MotorShield::motorCWrite(float duty_cycle, int direction) {
    int tick = (int)(periodTickVal * duty_cycle); 

    if (direction){
            TIM13->CCR1 = tick;
            TIM14->CCR1 = tick;
    }
    else {
            TIM13->CCR1 = tick;
            TIM14->CCR1 = 0;
    }
}
void MotorShield::motorDWrite(float duty_cycle, int direction) {
    int tick = (int)(periodTickVal * duty_cycle); 

    if (direction){
            TIM17->CCR1 = tick;
            TIM16->CCR1 = tick;
    }
    else {
            TIM17->CCR1 = tick;
            TIM16->CCR1 = 0;
    }
}

uint32_t MotorShield::readCurrentA() {
    return readADC1(0);
}
uint32_t MotorShield::readCurrentB() {
    return readADC1(1);
}
uint32_t MotorShield::readCurrentC() {
    return readADC2(1);
}
uint32_t MotorShield::readCurrentD() {
    return readADC2(0);
}

void MotorShield::changePeriod(int periodTicks){
    periodTickVal = periodTicks; 
    init();
}
