package com.powerstackers.velocity.common;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * @author Derek Helm
 */

public class VelRobot {


    /*
    Looking at the robot from above:
        -------------
        |1         2|
        |           |
        |           |
        |3         4|
        -------------
     */
    DcMotor drive1;
    DcMotor drive2;
    DcMotor drive3;
    DcMotor drive4;


    /**
     * Construct a Robot object.
     * @param mode The OpMode in which the robot is being used.
     */
    public VelRobot(OpMode mode) {
        drive1 = mode.hardwareMap.dcMotor.get("motorFrontLeft");
        drive2 = mode.hardwareMap.dcMotor.get("motorFrontRight");
        drive3 = mode.hardwareMap.dcMotor.get("motorBackLeft");
        drive4 = mode.hardwareMap.dcMotor.get("motorBackRight");

    }

    /**
     * Initialize the robot's servos and sensors.
     */
    public void initializeRobot() /*throws InterruptedException */{
        // TODO set motor modes
        stopMovement();
    }

    public void setMovement(double angle, double speed, double rotation) {
        
    }

    public void stopMovement() {
        drive1.setPower(0.0);
        drive2.setPower(0.0);
        drive3.setPower(0.0);
        drive4.setPower(0.0);
    }
}
