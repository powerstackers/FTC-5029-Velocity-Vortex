package com.powerstackers.velocity.common;

import com.powerstackers.velocity.common.enums.PublicEnums;
import com.powerstackers.velocity.common.enums.PublicEnums.AllianceColor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import static com.powerstackers.velocity.common.VelRobotConstants.BEACON_RESTING;

/**
 * @author Derek Helm
 */

public class VelRobotAuto {

    public OpMode mode;
    public VelJonsAlgo algorithm;

    //declarations
    /*
    Looking at the robot from above:
        -------------
        |1\\     //2|
        |           |
        |           |
        |3//     \\4|
        -------------
     */
    private DcMotor drive1 = null;
    private DcMotor drive2 = null;
    private DcMotor drive3 = null;
    private DcMotor drive4 = null;

    private Servo servoBeacon;

    private CRServo vexMotor;

    private GyroSensor sensorGyro;
    private ColorSensor sensorColor;


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
     *  Completely stop the drive motors.
     */
    public void stopMovement() {
        drive1.setPower(0.0);
        drive2.setPower(0.0);
        drive3.setPower(0.0);
        drive4.setPower(0.0);

        vexMotor.setPower(0);
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
