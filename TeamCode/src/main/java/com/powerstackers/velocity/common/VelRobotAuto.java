package com.powerstackers.velocity.common;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * @author Derek Helm
 */

public class VelRobotAuto {

    public OpMode mode;
    public VelJonsAlgo algorithm;

    //declarations
    private GyroSensor sensorGyro;


    /**
     * Construct a Robot object.
     * @param mode The OpMode in which the robot is being used.
     */
    public VelRobotAuto(OpMode mode) {

        //constructors
        this.mode = mode;
        algorithm = new VelJonsAlgo(this);


    }

    /**
     * Initialize the robot's servos and sensors.
     */
    public void initializeRobot() /*throws InterruptedException */{

        //init code for autonomous here vvv

    }


    public long getLeftEncoder() {
        return 0; //motorLeftA.getCurrentPosition();
    }

    public long getRightEncoder() {
        return 0; //motorRightA.getCurrentPosition();
    }

    public double getGyroHeading() {
        return  sensorGyro.getHeading();
    }

    public void calibrateGyro() {
        sensorGyro.calibrate();
    }

    public  boolean isGyrocalibrate() {
        return sensorGyro.isCalibrating();
    }

    public double getrawXGyro() {
        return sensorGyro.rawX();
    }

    public double getrawYGyro() {
        return sensorGyro.rawY();
    }

    public double getrawZGyro() {
        return sensorGyro.rawZ();
    }

    public OpMode getParentOpMode() {
        return mode;
    }

}
