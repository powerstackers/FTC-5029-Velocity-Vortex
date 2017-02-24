/*
 * Copyright (C) 2016 Powerstackers
 *
 * Basic configurations and capabilities of our robot.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

package com.powerstackers.velocity.common;

import com.powerstackers.velocity.common.enums.PublicEnums;
import com.powerstackers.velocity.common.enums.PublicEnums.MotorSetting;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.I2cAddr;

import java.sql.Array;
import java.util.Arrays;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

/**
 * Basic configurations for our robot. This class contains methods to make the robot do stuff.
 *
 * @author Cate Thomas
 */
@SuppressWarnings("unused")
public class VelRobot {

    protected final OpMode mode;
    /*
    Looking at the robot from above:
        ------F------
        |1\\     //2|
        |           |
        |           |
        |3//     \\4|
        -------------
     */
    DcMotor motorDrive1;
    DcMotor motorDrive2;
    DcMotor motorDrive3;
    DcMotor motorDrive4;

    private DcMotor motorPickup = null;
    private DcMotor motorShooter1;
    private DcMotor motorRLift;
    private DcMotor motorLLift;

    Servo servoBeacon = null;
    public Servo servoBallGrab = null;

    GyroSensor sensorGyro;
    public ColorSensor sensorColor;
    public ColorSensor sensorColorGroundL;
    public ColorSensor sensorColorGroundR;

    private final ElapsedTime timer = new ElapsedTime();

    /**
     * Construct a Robot object.
     *
     * @param mode The OpMode in which the robot is being used.
     */
    public VelRobot(OpMode mode) {
        this.mode = mode;
    }

    /**
     * Initialize the robot's servos and sensors.
     */
    public void initializeRobot() {
        motorDrive1 = mode.hardwareMap.dcMotor.get("motorFrontLeft");
        motorDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDrive2 = mode.hardwareMap.dcMotor.get("motorFrontRight");
        motorDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
//        motorDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDrive3 = mode.hardwareMap.dcMotor.get("motorBackLeft");
//        motorDrive3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDrive4 = mode.hardwareMap.dcMotor.get("motorBackRight");
        motorDrive4.setDirection(DcMotorSimple.Direction.REVERSE);
//        motorDrive4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRLift  = mode.hardwareMap.dcMotor.get("motorRightLift");
        motorRLift.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLLift  = mode.hardwareMap.dcMotor.get("motorLeftLift");
        motorPickup = mode.hardwareMap.dcMotor.get("motorBallPickup");

        motorShooter1 = mode.hardwareMap.dcMotor.get("motorShooter");
        motorShooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorShooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorShooter1.setMaxSpeed((int) (VelRobotConstants.MOTOR_SHOOTER_MAX_RPM * 0.74));
        motorShooter1.setDirection(DcMotorSimple.Direction.REVERSE);

        servoBallGrab = mode.hardwareMap.servo.get("servoBallGrab");
        servoBeacon   = mode.hardwareMap.servo.get("servoBeacon");

        sensorGyro = mode.hardwareMap.gyroSensor.get("sensorGyro");

        sensorGyro.calibrate();

        sensorColor         = mode.hardwareMap.colorSensor.get("sensorColor");
        sensorColorGroundL  = mode.hardwareMap.colorSensor.get("sensorColorGroundL");
        sensorColorGroundR  = mode.hardwareMap.colorSensor.get("sensorColorGroundR");

        sensorColor.setI2cAddress(I2cAddr.create7bit(0x1e)); //8bit 0x3c
        sensorColorGroundL.setI2cAddress(I2cAddr.create7bit(0x2e)); //8bit 0x5c
        sensorColorGroundR.setI2cAddress(I2cAddr.create7bit(0x26)); //8bit 0x4c

        sensorColor.enableLed(true);
        sensorColorGroundL.enableLed(true);
        sensorColorGroundR.enableLed(true);

        stopMovement();
        servoBeacon.setPosition(VelRobotConstants.BEACON_RESTING);
        servoBallGrab.setPosition(VelRobotConstants.SERVO_BALL_GRAB_STOWED);
        mode.telemetry.addData("Status", "Initialized");
    }

    /**
     * Get the revolutions per minute of the shooter motor. Eats up 100ms!
     * @return Double representing the rpm.
     */
    public double getShooterRPM() {

        int endEncoder;
        int startEncoder = motorShooter1.getCurrentPosition();
        timer.reset();

        //noinspection StatementWithEmptyBody
        while(timer.milliseconds() < 100) {}

        endEncoder = motorShooter1.getCurrentPosition();

        return (endEncoder - startEncoder) * (600.0/44.4);
    }

    /**
     * Set the direction of the particle pickup motor.
     * @param setting MotorSetting enum telling what setting to use.
     */
    public void setBallPickup(MotorSetting setting) {
        switch (setting) {
            case FORWARD:
                motorPickup.setPower(VelRobotConstants.MOTOR_PICKUP_POWER);
                break;
            case REVERSE:
                motorPickup.setPower(-VelRobotConstants.MOTOR_PICKUP_POWER);
                break;
            case STOP:
                motorPickup.setPower(0.0);
                break;
            default:
                motorPickup.setPower(0.0);
                break;
        }
    }

    /**
     * Set the shooter motors.
     * @param setting MotorSetting enum telling what setting to use.
     */
    public void setShooter(MotorSetting setting) {
        switch (setting) {
            case FORWARD:
                setShooterRpm(VelRobotConstants.MOTOR_SHOOTER_TARGET_RPM);
                break;
            case STOP:
                motorShooter1.setPower(0.0);
                break;
            default:
                motorShooter1.setPower(0.0);
                break;
        }
    }

    /**
     * Set the RPM of the shooter motor. Sets the speed as a percentage of the maximum RPM.
     * @param rpm RPM of motor that we want to set, can be positive or negative.
     */
    private void setShooterRpm(int rpm) {
        if (Math.abs(rpm) <= VelRobotConstants.MOTOR_SHOOTER_MAX_RPM) {
            motorShooter1.setPower( (double) rpm / VelRobotConstants.MOTOR_SHOOTER_MAX_RPM);
        } else {
            motorShooter1.setPower(rpm > 0? 1 : -1);
        }
    }

    /**
     * Set the lift motor.
     * @param setting MotorSetting telling which setting to use.
     */
    public void setLift(MotorSetting setting) {
        switch (setting) {
            case FORWARD:
                motorLLift.setPower(VelRobotConstants.MOTOR_LIFT_POWER);
                motorRLift.setPower(VelRobotConstants.MOTOR_LIFT_POWER);
                break;
            case REVERSE:
                motorLLift.setPower(-VelRobotConstants.MOTOR_LIFT_POWER);
                motorRLift.setPower(-VelRobotConstants.MOTOR_LIFT_POWER);
                break;
            case STOP:
                motorLLift.setPower(0.0);
                motorRLift.setPower(0.0);
                break;
            default:
                motorLLift.setPower(0.0);
                motorRLift.setPower(0.0);
                break;
        }
    }

    public void setBeaconTap(double position) {
        servoBeacon.setPosition(position);
    }

    /**
     * Set the movement speeds of all four motors, based on a desired angle, speed, and rotation
     * speed.
     *
     * @param angle The angle we want the robot to move, in radians, where "forward" is pi/2
     * @param speed The movement speed we want, ranging from -1:1
     * @param rotation The speed of rotation, ranging from -1:1
     */
    public void setMovement(double angle, double speed, double rotation, double scale) {
        // Shift angle by 45 degrees, since our drive train is x-shaped and not cross-shaped
        angle += PI/4;

        // Cut rotation in half because we don't want to spin THAT fast
        rotation *= 0.5;

        // Normalize magnitudes so that "straight forward" has a magnitude of 1
        speed *= sqrt(2);

        double sinDir = sin(angle);
        double cosDir = cos(angle);

        // None of this stuff should happen if the speed is 0.
        if (speed == 0.0 && rotation == 0.0) {
            stopMovement();
            return;
        }

        // Rotation is scaled down by 50% so that it doesn't completely cancel out any motors
        double multipliers[] = new double[4];
        multipliers[0] = (speed * sinDir) + rotation;
        multipliers[1] = (speed * cosDir) + rotation;
        multipliers[2] = (speed * -cosDir) + rotation;
        multipliers[3] = (speed * -sinDir) + rotation;

        double largest = abs(multipliers[0]);
        for (int i = 1; i < 4; i++) {
            if (abs(multipliers[i]) > largest)
                largest = abs(multipliers[i]);
        }

        // Only normalize multipliers if largest exceeds 1.0
        if(largest > 1.0) {
            for (int i = 0; i < 4; i++) {
                multipliers[i] = multipliers[i] / largest;
            }
        }

        // Scale if needed, 0.0 < scale < 1.0;
        for (int i = 0; i < 4; i++) {
            multipliers[i] = multipliers[i] * scale;
        }

        motorDrive1.setPower(multipliers[0]);
        motorDrive2.setPower(multipliers[1]);
        motorDrive3.setPower(multipliers[2]);
        motorDrive4.setPower(multipliers[3]);
    }

    /**
     *  Completely stop the drive motors.
     */
    public void stopMovement() {
        motorDrive1.setPower(0.0);
        motorDrive2.setPower(0.0);
        motorDrive3.setPower(0.0);
        motorDrive4.setPower(0.0);
    }

    /**
     * Completely stop all motors on the robot.
     */
    public void stopAllMotors() {
        motorDrive1.setPower(0.0);
        motorDrive2.setPower(0.0);
        motorDrive3.setPower(0.0);
        motorDrive4.setPower(0.0);

        motorShooter1.setPower(0.0);
        motorPickup.setPower(0.0);
        motorLLift.setPower(0.0);
        motorRLift.setPower(0.0);
    }

    public void MekMove(double x, double y, double turn) {
        double ypower = -y/3;
        double xpower = x/3;
        double rturn = -turn/3;

        motorDrive1.setPower(ypower + xpower - rturn);
        motorDrive2.setPower(ypower + xpower + rturn);
        motorDrive3.setPower(ypower - xpower - rturn);
        motorDrive4.setPower(ypower - xpower + rturn);
    }

    /**
     * Get direction of travel from the joystick.
     *
     * @param pad Gamepad to take control values from.
     * @return A direction of movement, in radians, where "forward" is pi/2
     */
    public static double mecDirectionFromJoystick(Gamepad pad) {
        return Math.atan2(-pad.left_stick_y, pad.left_stick_x);
    }

    /**
     *  Get the translation speed value from the joystick. If the joysticks are moved close enough
     *  to the center, the method will return 0 (meaning no movement).
     *
     * @param pad Gamepad to take control values from.
     * @return Speed ranging from 0:1
     */
    public static double mecSpeedFromJoystick(Gamepad pad) {
        // If the joystick is close enough to the middle, return a 0 (no movement)
        if (abs(pad.left_stick_x) < VelRobotConstants.MINIMUM_JOYSTICK_THRESHOLD
            && abs(pad.left_stick_y) < VelRobotConstants.MINIMUM_JOYSTICK_THRESHOLD){
            return 0.0;
        } else {
            return sqrt((pad.left_stick_y * pad.left_stick_y)
                + (pad.left_stick_x * pad.left_stick_x));
        }
    }

    /**
     *  Get the spin speed value from the joystick. If the joystick is moved close enough to the
     *  center, the method will return 0 (meaning no spin).
     *
     * @param pad Gamepad to take control values from.
     * @return Speed ranging from -1:1
     */
    public static double mecSpinFromJoystick(Gamepad pad) {
        return (abs(pad.right_stick_x) > VelRobotConstants.MINIMUM_JOYSTICK_THRESHOLD)
                ? pad.right_stick_x : 0.0;
    }

//    /**
//     * Tap the beacon on the correct side.
//     * @param allianceColor The color that we are currently playing as.
//     */
//    public void tapBeacon(PublicEnums.AllianceColor allianceColor) {
//        PublicEnums.AllianceColor dominantColor;
//        double positionBeaconServo;
//
//        // Detect the color shown on the beacon's left half, and record it.
//        if (sensorColor.red() > sensorColor.blue()) {
//            dominantColor = PublicEnums.AllianceColor.RED;
//        } else {
//            dominantColor = PublicEnums.AllianceColor.BLUE;
//        }
//
//        // Tap the correct side based on the dominant color.
//        if (dominantColor == allianceColor) {
//            positionBeaconServo = VelRobotConstants.BEACON_TAP_LEFT;
//        } else {
//            positionBeaconServo = VelRobotConstants.BEACON_TAP_RIGHT;
//        }
//
//        // Trim the servo value and set the servo position.
//        positionBeaconServo = trimServoValue(positionBeaconServo);
//        servoBeacon.setPosition(positionBeaconServo);
//    }

    /**
     * Trim a servo value between the minimum and maximum ranges.
     * @param servoValue Value to trim.
     * @return A raw double with the trimmed value.
     */
    private static double trimServoValue(double servoValue) {
        return Range.clip(servoValue, 0.0, 1.0);
    }

    /**
     * @return Int representation of the motor position.
     */
    public int getShooterEncVal() {
        return motorShooter1.getCurrentPosition();
    }

    public double getShooterPower(){
        return motorShooter1.getPower();
    }

    public long getDrive1Encoder() {
        return motorDrive1.getCurrentPosition();
    }

        public long getDrive2Encoder() {
        return motorDrive2.getCurrentPosition();
    }

    public long getDrive3Encoder() {
        return motorDrive3.getCurrentPosition();
    }

    public long getDrive4Encoder() {
        return motorDrive4.getCurrentPosition();
    }

    public int getEncoderShooter() throws InterruptedException {
        return motorShooter1.getCurrentPosition();
    }

    public double getBallGrabPosition() {
        return this.servoBallGrab.getPosition();
    }

//    public int getARGB() {
//        return sensorColor.argb();
//    }
//
//    public int getRed() {
//        return sensorColor.red();
//    }
//
//    public int getBlue() {
//        return sensorColor.blue();
//    }
//
//    public int getGreen() {
//        return sensorColor.green();
//    }
//
//    public int getAlpha() {
//        return sensorColor.alpha();
//    }

    // TODO Automatic ball feeder method
}
