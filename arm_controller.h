#ifndef ARM_CONTROLLER_HPP_
#define ARM_CONTROLLER_HPP_

#define ARM_DOWN 160 // 176 is horizontal; 177-180 is too far
#define ARM_UP 90
#define GRIPPER_CLOSE 0
#define GRIPPER_OPEN 180

class Arm_Controller {
    SoftwareServo servo_roll;
    SoftwareServo servo_pitch;
    SoftwareServo servo_grasp;
    
    int pins[3];          // for 3 motors of arm

public:
    Arm_Controller(int pin_roll, int pin_pitch, int pin_grasp) {
        // stores pin values
        pins[0] = pin_roll;
        pins[1] = pin_pitch;
        pins[2] = pin_grasp;
        
        // sets pins as output of Arduino
        servo_roll.attach(pin_roll);
        servo_pitch.attach(pin_pitch);
        servo_grasp.attach(pin_grasp);
    }

    // changes roll of first servo
    void change_roll(int val) {
        servo_roll.write(val);
    }
    
    // changes pitch of 2nd servo
    void change_pitch(int val) {
        servo_pitch.write(val);
    }
    
    // opens or closes gripper
    void change_grasp(int val) {
        servo_grasp.write(val);
    }
    

};

#endif
