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

import com.powerstackers.velocity.common.enums.PublicEnums.GrabberSetting;
import com.powerstackers.velocity.common.enums.PublicEnums.MotorSetting;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.PI;
import static java.lang.Math.abs;

/**
 * Basic configurations for our robot. This class contains methods to make the robot do stuff.
 *
 * @author Cate Thomas
 */

public class VelRobot {



    protected OpMode mode;
    /*
    Looking at the robot from above:
        -------------
        |1\\     //2|
        |           |
        |           |
        |3//     \\4|
        -------------
     */
    protected DcMotor motorDrive1;
    protected DcMotor motorDrive2;
    protected DcMotor motorDrive3;
    protected DcMotor motorDrive4;
    protected DcMotor motorPickup;
    protected DcMotor motorShooter1;
    protected DcMotor motorShooter2;
    protected DcMotor motorLift;


    protected Servo servoBeacon;
    protected Servo servoBallGrab;

    protected CRServo vexMotor;

    protected GyroSensor sensorGyro;
    protected ColorSensor sensorColor;

    private boolean ENGAGE_STUPID_MODE = false;
    private ElapsedTime timer = new ElapsedTime();

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
    public void initializeRobot() /*throws InterruptedException */{
        // TODO set motor modes
        mode.telemetry.addData("Status", "Initialized");
        motorDrive1 = mode.hardwareMap.dcMotor.get("motorFrontLeft");
        motorDrive2 = mode.hardwareMap.dcMotor.get("motorFrontRight");
        motorDrive3 = mode.hardwareMap.dcMotor.get("motorBackLeft");
        motorDrive4 = mode.hardwareMap.dcMotor.get("motorBackRight");
        motorLift = mode.hardwareMap.dcMotor.get("motorLift");
        // Don't configure these motors in stupid mode.
        if (!ENGAGE_STUPID_MODE) {
            motorPickup = mode.hardwareMap.dcMotor.get("motorBallPickup");
            motorShooter1 = mode.hardwareMap.dcMotor.get("motorShooter1");
            motorShooter2 = mode.hardwareMap.dcMotor.get("motorShooter2");

            vexMotor = mode.hardwareMap.crservo.get("vexServo");
            servoBallGrab = mode.hardwareMap.servo.get("servoBallGrab");
        }




        stopMovement();
        if (!ENGAGE_STUPID_MODE)
            servoBallGrab.setPosition(VelRobotConstants.SERVO_BALL_GRAB_STOWED);
    }

    /**
     * Get the revolutions per minute of the shooter motor.
     * @return
     */
    public double getShooterRPM() {

        int endEncoder;
        int startEncoder = motorShooter1.getCurrentPosition();
        timer.reset();

        while(timer.milliseconds() < 100) {}

        endEncoder = motorShooter1.getCurrentPosition();

        return (endEncoder - startEncoder) * (600.0/44.4);

    }

    /**
     * Set the direction of the particle pickup motor.
     * @param setting MotorSetting enum telling what setting to use.
     */
    public void setBallPickup(MotorSetting setting) {
        if (ENGAGE_STUPID_MODE) return;
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
                motorShooter1.setPower(VelRobotConstants.MOTOR_SHOOTER_POWER);
                motorShooter2.setPower(-VelRobotConstants.MOTOR_SHOOTER_POWER);
                break;
            case STOP:
                motorShooter1.setPower(0.0);
                motorShooter2.setPower(0.0);
                break;
            default:
                motorShooter1.setPower(0.0);
                motorShooter2.setPower(0.0);
                break;
        }
    }

    /**
     * Set the lift motor.
     * @param setting MotorSetting telling which setting to use.
     */
    public void setLift(MotorSetting setting) {
        if (ENGAGE_STUPID_MODE) return;
        switch (setting) {
            case FORWARD:
                motorLift.setPower(VelRobotConstants.MOTOR_LIFT_POWER);
                break;
            case REVERSE:
                motorLift.setPower(-VelRobotConstants.MOTOR_LIFT_POWER);
                break;
            case STOP:
                motorLift.setPower(0.0);
                break;
            default:
                motorLift.setPower(0.0);
                break;
        }
    }

    /**
     * Release the ball grabber.
     */
    public void releaseBallGrab() {
        if (ENGAGE_STUPID_MODE) return;
        servoBallGrab.setPosition(VelRobotConstants.SERVO_BALL_GRAB_OPEN);
    }

    /**
     * Set the cap ball grabber.
     * @param setting GrabberSetting telling which position to set to.
     */
    public void setBallGrab(GrabberSetting setting) {
        if (ENGAGE_STUPID_MODE) return;
        servoBallGrab.setPosition(setting == GrabberSetting.LOOSE?
                VelRobotConstants.SERVO_BALL_GRAB_OPEN : VelRobotConstants.SERVO_BALL_GRAB_TIGHT);
    }



    /**
     * Set the movement speeds of all four motors, based on a desired angle, speed, and rotation
     * speed.
     *
     * @param angle The angle we want the robot to move, in radians, where "forward" is pi/2
     * @param speed The movement speed we want, ranging from -1:1
     * @param rotation The speed of rotation, ranging from -1:1
     */
    public void setMovement(double angle, double speed, double rotation) {

        // None of this stuff should happen if the speed is 0.
        if (speed == 0.0 && rotation == 0.0) {
            stopMovement();
            return;
        }

        // Rotation is scaled down by 50% so that it doesn't completely cancel out any motors
        double multipliers[] = new double[4];
        multipliers[0] = (speed * Math.sin(angle + (PI/4)))  + (rotation * 0.5);
        multipliers[1] = (speed * Math.cos(angle + (PI/4)))  + (rotation * 0.5);
        multipliers[2] = (speed * -Math.cos(angle + (PI/4))) + (rotation * 0.5);
        multipliers[3] = (speed * -Math.sin(angle + (PI/4))) + (rotation * 0.5);

        double largest = abs(multipliers[0]);
        for (int i = 1; i < 4; i++) {
            if (abs(multipliers[i]) > largest)
                largest = abs(multipliers[i]);
        }

        for (int i = 0; i < 4; i++) {
            multipliers[i] = multipliers[i] / largest;
        }

        motorDrive1.setPower(multipliers[0]);
        motorDrive2.setPower(multipliers[1]);
        motorDrive3.setPower(multipliers[2]);
        motorDrive4.setPower(multipliers[3]);
    }

    /**
     * set vexmotor power
     */
    public void vexPower(double power) {
        if (ENGAGE_STUPID_MODE) return;
        vexMotor.setPower(power);
    }

    /**
     *  Completely stop the drive motors.
     */
    public void stopMovement() {
        motorDrive1.setPower(0.0);
        motorDrive2.setPower(0.0);
        motorDrive3.setPower(0.0);
        motorDrive4.setPower(0.0);

        vexMotor.setPower(0);
    }

    /**
     * Allows robot to go all the way froward and backwards on the y-axis
     *
     * @param pad Gamepad to take control values from.
     * @return A directon of movement, in radians, where "forward" is pi/2
     */
    public static double mecDirectionFromJoystick(Gamepad pad) {
        double x = pad.left_stick_x;
        double y = pad.left_stick_y;   // The Y stick is inverted

        // If x is exactly 0, atan will be undefined. In that case, our angle is either 90 or 270.
        if (x == 0) {
            return ((y > 0)? PI / 2 : (PI * 3) / 2);
        } else {
            double atan = Math.atan(y / x);

            // Make sure the angle is in the right quadrant.
            if (x > 0) {
                return ((y > 0) ? atan : atan + (PI * 2));
            } else {
                return atan + PI;
            }
        }
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
            return Math.sqrt((pad.left_stick_y * pad.left_stick_y)
                + (pad.left_stick_x * pad.left_stick_x));
        }
    }

    /**
     *  Get the spin speed value from the joystick. If the joystick is moved close enough to the
     *  center, the method will return 0 (meaning no spin).
     *  It's important to note that pushing the stick to the left should mean a counter-clockwise
     *  rotation; meaning, the stick needs to be reversed.
     *
     * @param pad Gamepad to take control values from.
     * @return Speed ranging from -1:1
     */
    public static double mecSpinFromJoystick(Gamepad pad) {
        return (double) ((abs(pad.right_stick_x) > VelRobotConstants.MINIMUM_JOYSTICK_THRESHOLD)
                ? -pad.right_stick_x : 0.0);
    }

    public int getShooterEncVal() {
        return motorShooter1.getCurrentPosition();
    }
}
