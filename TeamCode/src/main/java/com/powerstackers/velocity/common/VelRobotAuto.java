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

    /**
     * Trim a servo value between the minimum and maximum ranges.
     * @param servoValue Value to trim.
     * @return A raw double with the trimmed value.
     */
    private double trimServoValue(double servoValue) {
        return Range.clip(servoValue, 0.0, 1.0);
    }

    /**
     * Tap the beacon on the correct side.
     * @param allianceColor The color that we are currently playing as.
     */
    public void tapBeacon(AllianceColor allianceColor) {

        AllianceColor dominantColor;
        double positionBeaconServo;

        // Detect the color shown on the beacon's left half, and record it.
        if (sensorColor.red() > sensorColor.blue()) {
            dominantColor = AllianceColor.RED;
        } else {
            dominantColor = AllianceColor.BLUE;
        }

        // Tap the correct side based on the dominant color.
        if (dominantColor == allianceColor) {
            positionBeaconServo = VelRobotConstants.BEACON_TAP_LEFT;
        } else {
            positionBeaconServo = VelRobotConstants.BEACON_TAP_RIGHT;
        }

//         Trim the servo value and set the servo position.
        positionBeaconServo = trimServoValue(positionBeaconServo);
        servoBeacon.setPosition(positionBeaconServo);
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
