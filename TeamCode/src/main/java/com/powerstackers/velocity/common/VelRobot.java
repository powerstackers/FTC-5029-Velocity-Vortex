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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.I2cAddr;

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
    public DcMotor motorShooter1;
    private DcMotor motorRLift;
    private DcMotor motorLLift;

    Servo servoBeaconRight = null;
    Servo servoBeaconLeft = null;
    public Servo servoBallGrab = null;
    public Servo servoShoot = null;
    public double matColorVal = 0;
    public int startDirection = 0;

    GyroSensor sensorGyro;
    public ColorSensor sensorColor;
    public UltrasonicSensor rightBeaconUS = null;
    public UltrasonicSensor leftBeaconUS = null;
    public OpticalDistanceSensor groundODS = null;
    public ColorSensor sensorColorGroundL;
    public ColorSensor sensorColorGroundR;
    public PublicEnums.Direction robotDirection = PublicEnums.Direction.N;
    public MiniPID shooterPID = new MiniPID(0.03, 0, 0.01);

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
    public void initializeRobot() throws InterruptedException {
        mode.telemetry.addData("Status: ", "Initialization Started");
        mode.telemetry.update();
        mode.telemetry.addData("Status: ", "Initalizing");
        mode.telemetry.update();
        motorDrive1 = mode.hardwareMap.dcMotor.get("motorFrontLeft");
        //motorDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDrive2 = mode.hardwareMap.dcMotor.get("motorFrontRight");
        //motorDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDrive3 = mode.hardwareMap.dcMotor.get("motorBackLeft");
        //motorDrive3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDrive4 = mode.hardwareMap.dcMotor.get("motorBackRight");
//        motorDrive4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRLift = mode.hardwareMap.dcMotor.get("motorRightLift");
        motorRLift.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLLift = mode.hardwareMap.dcMotor.get("motorLeftLift");
        motorPickup = mode.hardwareMap.dcMotor.get("motorBallPickup");

        motorShooter1 = mode.hardwareMap.dcMotor.get("motorShooter");
        motorShooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //motorShooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        motorShooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        servoShoot = mode.hardwareMap.servo.get("servoShoot");
        servoBallGrab = mode.hardwareMap.servo.get("servoBallGrab");
        servoBeaconRight = mode.hardwareMap.servo.get("servoBeaconRight");
        servoBeaconLeft = mode.hardwareMap.servo.get("servoBeaconLeft");
        sensorGyro = mode.hardwareMap.gyroSensor.get("sensorGyro");

//        mode.telemetry.addData("Gyro: ", "Gyro Calibration Started");
//        mode.telemetry.update();
//        sensorGyro.calibrate();
//        int gyroCalbCount = 0;
//        while (sensorGyro.isCalibrating()) {
//            mode.telemetry.addData("Gyro: ", "Gyro is Calibrating");
//            mode.telemetry.addData("Gyro Calb Count: ", gyroCalbCount);
//            gyroCalbCount++;
//            mode.telemetry.update();
//            Thread.sleep(10);
//        }
//        mode.telemetry.addData("Gyro: ", "Gyro Calibration Finished");
//        mode.telemetry.update();
        servoShoot.setPosition( VelRobotConstants.SHOOT_SERVO_CLOSED);
        sensorColor = mode.hardwareMap.colorSensor.get("sensorColor");
        sensorColorGroundL = mode.hardwareMap.colorSensor.get("sensorColorGroundL");
        sensorColorGroundR = mode.hardwareMap.colorSensor.get("sensorColorGroundR");

        sensorColor.setI2cAddress(I2cAddr.create7bit(0x1e)); //8bit 0x3c
        sensorColorGroundL.setI2cAddress(I2cAddr.create7bit(0x2e)); //8bit 0x5c
        sensorColorGroundR.setI2cAddress(I2cAddr.create7bit(0x26)); //8bit 0x4c
        groundODS = mode.hardwareMap.opticalDistanceSensor.get("ODS");
        rightBeaconUS = mode.hardwareMap.ultrasonicSensor.get("RUS");
        leftBeaconUS = mode.hardwareMap.ultrasonicSensor.get("LUS");
        sensorColor.enableLed(true);
        sensorColorGroundL.enableLed(true);
        sensorColorGroundR.enableLed(true);
        //motorShooter1.setMaxSpeed((int) (VelRobotConstants.MOTOR_SHOOTER_MAX_RPM*0.74));
        stopMovement();
        matColorVal = groundODS.getLightDetected();
        shooterPID.setOutputLimits(0.0, 1.0);
        beaconServoReset();
//        servoBallGrab.setPosition(0.493);
        servoBallGrab.setPosition(0.5);
        mode.telemetry.addData("Status: ", "Initialized");
        mode.telemetry.update();
    }

    public void displayDirection() {
        mode.telemetry.addData("Robot Direction:", robotDirection);
        mode.telemetry.update();
    }

    /**
     * Get the revolutions per minute of the shooter motor. Eats up 100ms!
     *
     * @return Double representing the rpm.
     */

    public boolean isShooterRunning() {
        if (motorShooter1.getPower() > 0) {
            return true;
        } else {
            return false;
        }
    }

    public void beaconServoReset() {
        servoBeaconRight.setPosition(VelRobotConstants.BEACON_RIGHT_BACK);
        servoBeaconLeft.setPosition(VelRobotConstants.BEACON_LEFT_BACK);
    }

    public double getShooterRPM() {

        int endEncoder;
        int startEncoder = motorShooter1.getCurrentPosition();
         timer.reset();

        //noinspection StatementWithEmptyBody
        while (timer.milliseconds() < 100) {
        }

        endEncoder = motorShooter1.getCurrentPosition();

        return (endEncoder - startEncoder) * (600.0 / 44.4);
    }

    /**
     * Set the direction of the particle pickup motor.
     *
     * @param setting MotorSetting enum telling what setting to use.
     */
    public void setBallPickup(MotorSetting setting) {
        switch (setting) {
            case FORWARD:
                motorPickup.setPower(-VelRobotConstants.MOTOR_PICKUP_POWER/8);
                break;
            case REVERSE:
                motorPickup.setPower(VelRobotConstants.MOTOR_PICKUP_POWER);
                break;
            case STOP:
                motorPickup.setPower(0.0);
                break;
            default:
                motorPickup.setPower(0.0);
                break;
        }
    }
    public void leftBeaconPosition(PublicEnums.BeaconPostion beaconPostion){
        if (beaconPostion == PublicEnums.BeaconPostion.OUT){
            servoBeaconLeft.setPosition(VelRobotConstants.BEACON_LEFT_FORWARD);
        }
        else{
            servoBeaconLeft.setPosition(VelRobotConstants.BEACON_LEFT_BACK);

        }
    }
    public void rightBeaconPosition(PublicEnums.BeaconPostion beaconPostion){
        if (beaconPostion == PublicEnums.BeaconPostion.OUT){
            servoBeaconRight.setPosition(VelRobotConstants.BEACON_RIGHT_FORWARD);
        }
        else{
            servoBeaconRight.setPosition(VelRobotConstants.BEACON_RIGHT_BACK);

        }
    }
    public void directionChange(PublicEnums.Direction direction) {
        robotDirection = direction;

        switch (direction) {
            case N:
                motorDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
                motorDrive2.setDirection(DcMotorSimple.Direction.FORWARD);
                motorDrive3.setDirection(DcMotorSimple.Direction.FORWARD);
                motorDrive4.setDirection(DcMotorSimple.Direction.FORWARD);
                mode.telemetry.addData("Robot Direction:", "N");
                break;
            case E:
                motorDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
                motorDrive2.setDirection(DcMotorSimple.Direction.FORWARD);
                motorDrive3.setDirection(DcMotorSimple.Direction.REVERSE);
                motorDrive4.setDirection(DcMotorSimple.Direction.REVERSE);

                mode.telemetry.addData("Robot Direction:", "E");

                break;
            case S:
                motorDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
                motorDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
                motorDrive3.setDirection(DcMotorSimple.Direction.REVERSE);
                motorDrive4.setDirection(DcMotorSimple.Direction.REVERSE);
                mode.telemetry.addData("Robot Direction:", "S");

                break;
            case W:
                motorDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
                motorDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
                motorDrive3.setDirection(DcMotorSimple.Direction.FORWARD);
                motorDrive4.setDirection(DcMotorSimple.Direction.FORWARD);

                mode.telemetry.addData("Robot Direction:", "W");

                break;
            default:
                motorDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
                motorDrive2.setDirection(DcMotorSimple.Direction.FORWARD);
                motorDrive3.setDirection(DcMotorSimple.Direction.FORWARD);
                motorDrive4.setDirection(DcMotorSimple.Direction.FORWARD);
                mode.telemetry.addData("Robot Direction:", "N");
                break;

        }
        mode.telemetry.update();
    }

    /**
     * Set the shooter motors.
     *
     * @param setting MotorSetting enum telling what setting to use.
     */
    public void setShooter(MotorSetting setting) {
        switch (setting) {
            case FORWARD:
                double outputSpeed = shooterPID.getOutput(getShooterRPM(), VelRobotConstants.MOTOR_SHOOTER_TARGET_RPM);
                motorShooter1.setPower(outputSpeed);
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
//    private void setShooterRpm(int rpm) {
//        double shootRpm = getShooterRPM();
//
//        // Uses somehting called a Schmitt trigger. Has an upper and lower threshold. Should stop us
//        // from bouncing around.
//        // If RPM is below the lower threshold, add speed. If it is above the upper threshold,
//        // subtract speed. If it is between the thresholds, do nothing. Remember, don't go outside
//        // the limits of our motor values.
//        if (shootRpm < rpm - VelRobotConstants.SCHMITT_LOWER
//                && motorShooter1.getPower() != 1.0) {
//            motorShooter1.setPower(motorShooter1.getPower() + 0.05);
//        } else if (shootRpm > rpm + VelRobotConstants.SCHMITT_UPPER
//                && motorShooter1.getPower() != 0.0) {
//            motorShooter1.setPower(motorShooter1.getPower() - 0.05);
//        }
//    }

    /**
     * Set the RPM of the shooter motor. Sets the speed as a percentage of the maximum RPM.
     *
     * @param rpm RPM of motor that we want to set, can be positive or negative.
     */
    private void setShooterRpm(int rpm) {
        motorShooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (Math.abs(rpm) <= VelRobotConstants.MOTOR_SHOOTER_MAX_RPM) {
            motorShooter1.setPower((double) rpm / VelRobotConstants.MOTOR_SHOOTER_MAX_RPM);
        } else {
            motorShooter1.setPower(rpm > 0 ? 1.0 : -1.0);
        }
//
//        if (getShooterRPM() > 650){
//        motorShooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        if (Math.abs(rpm) <= VelRobotConstants.MOTOR_SHOOTER_MAX_RPM) {
//            motorShooter1.setPower((double) rpm / VelRobotConstants.MOTOR_SHOOTER_MAX_RPM);
//        } else {
//            motorShooter1.setPower(rpm > 0 ? 1.0 : -1.0);
//        }}
    }

    /**
     * Set the lift motor.
     *
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
        servoBeaconRight.setPosition(position);
    }

    /**
     * Set the movement speeds of all four motors, based on a desired angle, speed, and rotation
     * speed.
     *
     * @param angle    The angle we want the robot to move, in radians, where "forward" is pi/2
     * @param speed    The movement speed we want, ranging from -1:1
     * @param rotation The speed of rotation, ranging from -1:1
     */
    public void setMovement(double angle, double speed, double rotation, double scale) {
        // Shift angle by 45 degrees, since our drive train is x-shaped and not cross-shaped
        angle += PI / 4;

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
        if (largest > 1.0) {
            for (int i = 0; i < 4; i++) {
                multipliers[i] = multipliers[i] / largest;
            }
        }

        // Scale if needed, 0.0 < scale < 1.0;
//        for (int i = 0; i < 4; i++) {
//            multipliers[i] = multipliers[i] * scale;
//        }

        // TODO Fix wiring. Motors 2 and 4 are plugged into the wrong motor ports.
        motorDrive1.setPower(multipliers[0] * scale);
        motorDrive4.setPower(multipliers[1] * scale);
        motorDrive3.setPower(multipliers[2] * scale);
        motorDrive2.setPower(multipliers[3] * scale);
    }

    public void driveToLine(double angle, double speed, PublicEnums.GyroCorrection gyroCorrection, PublicEnums.BeaconNumber beaconNumber) {

        // Shift angle by 45 degrees, since our drive train is x-shaped and not cross-shaped
        angle += PI / 4;

        // Cut rotation in half because we don't want to spin THAT fast

        // Normalize magnitudes so that "straight forward" has a magnitude of 1
        speed *= sqrt(2);

        double sinDir = sin(angle);
        double cosDir = cos(angle);

        // None of this stuff should happen if the speed is 0.

        // Rotation is scaled down by 50% so that it doesn't completely cancel out any motors
        double multipliers[] = new double[4];
        multipliers[0] = (speed * sinDir);
        multipliers[1] = (speed * cosDir);
        multipliers[2] = (speed * -cosDir);
        multipliers[3] = (speed * -sinDir);

        double largest = abs(multipliers[0]);
        for (int i = 1; i < 4; i++) {
            if (abs(multipliers[i]) > largest)
                largest = abs(multipliers[i]);
        }

        // Only normalize multipliers if largest exceeds 1.0
        if (largest > 1.0) {
            for (int i = 0; i < 4; i++) {
                multipliers[i] = multipliers[i] / largest;
            }
        }

        // Scale if needed, 0.0 < scale < 1.0;
//        for (int i = 0; i < 4; i++) {
//            multipliers[i] = multipliers[i] * scale;
//        }
//        int x = 0;
//        while (isThereMat() && mode.opModeIsActive() && x == 0) {
//            if (gyroCorrection == PublicEnums.GyroCorrection.YES) {
//                int currentHead = startDirection;
//                // TODO Fix wiring. Motors 2 and 4 are plugged into the wrong motor ports.
//
////if (beaconNumber == PublicEnums.BeaconNumber.ONE &&(rightBeaconUS.getUltrasonicLevel() <= 10 || leftBeaconUS.getUltrasonicLevel() <= 10)){
////        x++;
////}
//                if (getGyroHeading() < currentHead) {
//                    motorDrive1.setPower(multipliers[0]);
//                    motorDrive3.setPower(multipliers[2]);
//                    motorDrive4.setPower(multipliers[1] / 1.5);
//                    motorDrive2.setPower(multipliers[3] / 1.5);
//                } else if (getGyroHeading() > currentHead) {
//                    motorDrive1.setPower(multipliers[0] / 1.5);
//                    motorDrive3.setPower(multipliers[2] / 1.5);
//                    motorDrive4.setPower(multipliers[1]);
//                    motorDrive2.setPower(multipliers[3]);
//                } else {
//                    motorDrive1.setPower(multipliers[0]);
//                    motorDrive3.setPower(multipliers[2]);
//                    motorDrive4.setPower(multipliers[1]);
//                    motorDrive2.setPower(multipliers[3]);
//                }
//
//            } else {
//
//                motorDrive1.setPower(multipliers[0]);
//                motorDrive3.setPower(multipliers[2]);
//                motorDrive4.setPower(multipliers[1]);
//                motorDrive2.setPower(multipliers[3]);
//            }
//        }
//        if (x ==1) {
//            driveToLine(VelRobotConstants.DIRECTION_WEST, 0.8, PublicEnums.GyroCorrection.NO, PublicEnums.BeaconNumber.TWO);
//        }
        double startGyroVal = sensorGyro.getHeading();
        setMovement(angle, speed, 0, 1);
        while (isThereMat()) {

        }
        stopMovement();
    }
//
//    public void driveWithUS(double angle, double speed, double target) {
//
//        // Shift angle by 45 degrees, since our drive train is x-shaped and not cross-shaped
//        angle += PI / 4;
//
//        // Cut rotation in half because we don't want to spin THAT fast
//
//        // Normalize magnitudes so that "straight forward" has a magnitude of 1
//        speed *= sqrt(2);
//
//        double sinDir = sin(angle);
//        double cosDir = cos(angle);
//
//        // None of this stuff should happen if the speed is 0.
//
//        // Rotation is scaled down by 50% so that it doesn't completely cancel out any motors
//        double multipliers[] = new double[4];
//        multipliers[0] = (speed * sinDir);
//        multipliers[1] = (speed * cosDir);
//        multipliers[2] = (speed * -cosDir);
//        multipliers[3] = (speed * -sinDir);
//
//        double largest = abs(multipliers[0]);
//        for (int i = 1; i < 4; i++) {
//            if (abs(multipliers[i]) > largest)
//                largest = abs(multipliers[i]);
//        }
//
//        // Only normalize multipliers if largest exceeds 1.0
//        if (largest > 1.0) {
//            for (int i = 0; i < 4; i++) {
//                multipliers[i] = multipliers[i] / largest;
//            }
//        }
//
//        // Scale if needed, 0.0 < scale < 1.0;
////        for (int i = 0; i < 4; i++) {
////            multipliers[i] = multipliers[i] * scale;
////        }
//        int currentHead = startDirection;
//        // TODO Fix wiring. Motors 2 and 4 are plugged into the wrong motor ports.
//        double correctionScaleRight;
//        double correctionScaleLeft;
//// any errors with misalignment will get fixed when the robot squares on the wall
//        while (leftBeaconUS.getUltrasonicLevel() > target && rightBeaconUS.getUltrasonicLevel() > target && mode.opModeIsActive()) {
//            if (leftBeaconUS.getUltrasonicLevel() < rightBeaconUS.getUltrasonicLevel()) {
//                correctionScaleLeft = 1.5;
//                correctionScaleRight = 1;
//            } else if (leftBeaconUS.getUltrasonicLevel() > rightBeaconUS.getUltrasonicLevel()) {
//                correctionScaleLeft = 1.5;
//                correctionScaleRight = 1;
//            } else {
//                correctionScaleLeft = 1;
//                correctionScaleRight = 1;
//            }
//            motorDrive1.setPower(multipliers[0] / correctionScaleLeft);
//            motorDrive3.setPower(multipliers[2] / correctionScaleLeft);
//            motorDrive4.setPower(multipliers[1] / correctionScaleRight);
//            motorDrive2.setPower(multipliers[3] / correctionScaleRight);
//            mode.telemetry.addData("Robot Heading", sensorGyro.getHeading());
//            mode.telemetry.update();
//        }
//
//        stopMovement();
//    }

    public double getGroundLight() {
        return groundODS.getLightDetected();
    }

    public boolean isThereMat() {

        if (getGroundLight() - .3 <= matColorVal) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Completely stop the drive motors.
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
     * Get the translation speed value from the joystick. If the joysticks are moved close enough
     * to the center, the method will return 0 (meaning no movement).
     *
     * @param pad Gamepad to take control values from.
     * @return Speed ranging from 0:1
     */
    public static double mecSpeedFromJoystick(Gamepad pad) {
        // If the joystick is close enough to the middle, return a 0 (no movement)
        if (abs(pad.left_stick_x) < VelRobotConstants.MINIMUM_JOYSTICK_THRESHOLD
                && abs(pad.left_stick_y) < VelRobotConstants.MINIMUM_JOYSTICK_THRESHOLD) {
            return 0.0;
        } else {
            return sqrt((pad.left_stick_y * pad.left_stick_y)
                    + (pad.left_stick_x * pad.left_stick_x));
        }
    }

    /**
     * Get the spin speed value from the joystick. If the joystick is moved close enough to the
     * center, the method will return 0 (meaning no spin).
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
//        servoBeaconRight.setPosition(positionBeaconServo);
//    }

    /**
     * Trim a servo value between the minimum and maximum ranges.
     *
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

    public double getShooterPower() {
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

    //public double getBallGrabPosition() {
//        return this.servoBallGrab.getPosition();
//    }

    public double getGyroHeading() {
        return sensorGyro.getHeading();
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
