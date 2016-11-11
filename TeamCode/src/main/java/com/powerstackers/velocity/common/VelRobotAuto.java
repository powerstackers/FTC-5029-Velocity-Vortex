package com.powerstackers.velocity.common;

import com.powerstackers.velocity.common.enums.PublicEnums;
import com.powerstackers.velocity.common.enums.PublicEnums.AllianceColor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import static com.powerstackers.velocity.common.VelRobotConstants.BEACON_RESTING;

/**
 * Basic configurations for our robot in autonomous mode. All the functionality of a teleop bot,
 * and more!
 * 
 * @author Derek Helm
 */

public class VelRobotAuto extends VelRobot {

    public VelJonsAlgo algorithm;

    /**
     * Construct a Robot object.
     * @param mode The OpMode in which the robot is being used.
     */
    public VelRobotAuto(OpMode mode) {

        super(mode);
        //constructors
        algorithm = new VelJonsAlgo(this);
    }

    /**
     * Initialize the robot's servos and sensors.
     */
    public void initializeRobot() /*throws InterruptedException */{
        // TODO Is there any difference between autonomous initialize and teleop initialize?
        //init code for autonomous here vvv
        mode.telemetry.addData("Status", "Initialized");
        drive1 = mode.hardwareMap.dcMotor.get("motorFrontLeft");
        drive2 = mode.hardwareMap.dcMotor.get("motorFrontRight");
        drive3 = mode.hardwareMap.dcMotor.get("motorBackLeft");
        drive4 = mode.hardwareMap.dcMotor.get("motorBackRight");

        servoBeacon = mode.hardwareMap.servo.get("servoBeacon");
        servoBeacon.setPosition(BEACON_RESTING);

        vexMotor = mode.hardwareMap.crservo.get("vexServo");

        sensorColor = mode.hardwareMap.colorSensor.get("sensorColor");
        sensorColor.enableLed(true);

        stopMovement();

    }




    public long getLeftEncoder() {
        return drive1.getCurrentPosition();
    }

    public long getRightEncoder() {
        return drive2.getCurrentPosition();
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
