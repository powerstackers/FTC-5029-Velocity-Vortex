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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;

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
    protected DcMotor drive1 = null;
    protected DcMotor drive2 = null;
    protected DcMotor drive3 = null;
    protected DcMotor drive4 = null;

    protected Servo servoBeacon;

    protected CRServo vexMotor;

    protected GyroSensor sensorGyro;
    protected ColorSensor sensorColor;


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
        drive1 = mode.hardwareMap.dcMotor.get("motorFrontLeft");
        drive2 = mode.hardwareMap.dcMotor.get("motorFrontRight");
        drive3 = mode.hardwareMap.dcMotor.get("motorBackLeft");
        drive4 = mode.hardwareMap.dcMotor.get("motorBackRight");

        vexMotor = mode.hardwareMap.crservo.get("vexServo");
        stopMovement();
    }

    /**
     * Just a simple absolute value function.
     * @param val Value to get absolute value of.   not neccasary due to math.abs existing
     * @return Absolute value of the input.
     */
//    private static double abs(double val) {return (val > 0)? val : -val;}

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
        if (speed == 0.0) {
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

        drive1.setPower(multipliers[0]);
        drive2.setPower(multipliers[1]);
        drive3.setPower(multipliers[2]);
        drive4.setPower(multipliers[3]);
    }

    /**
     * method 2 for meccanum
     */
    public  void magicMecanum(Gamepad gamepad) {

        double Ch1, Ch3, Ch4;

        Ch1 = (-(gamepad.right_stick_x));
        Ch3 = gamepad.left_stick_y;
        Ch4 = (-(gamepad.left_stick_x));

        drive1.setPower(-(Ch3 + Ch1 + Ch4));
        drive2.setPower(Ch3 - Ch1 - Ch4);
        drive3.setPower(-(Ch3 + Ch1 - Ch4));
        drive4.setPower(Ch3 - Ch1 + Ch4);
    }

    /**
     * set vexmotor power
     */
    public void vexPower(double power) {
        vexMotor.setPower(power);
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
     * Allows robot to go all the way froward and backwards on the y-axis
     *
     * @param pad Gamepad to take control values from.
     * @return A directon of movement, in radians, where "forward" is pi/2
     */
    public static double mecDirectionFromJoystick(Gamepad pad) {
        double x = pad.left_stick_x;
        double y = -pad.left_stick_y;   // The Y stick is inverted

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

    /**
     * get VexMotor power
     */
    public double getVexPower() {
        return vexMotor.getPower();
    }

// TODO Collaps all these into one method?
    /**
     * get moter telemetry
     */
    public double getDrive1Power() {
        return  drive1.getPower();
    }
    /**
     * get moter telemetry
     */
    public double getDrive2Power() {
        return  drive2.getPower();
    }
    /**
     * get moter telemetry
     */
    public double getDrive3Power() {
        return  drive3.getPower();
    }
    /**
     * get moter telemetry
     */
    public double getDrive4Power() {
        return  drive4.getPower();
    }

    /**
     * get port nuber
     */
    public int getDrive1Port() {
        return drive1.getPortNumber();
    }
    /**
     * get port nuber
     */
    public int getDrive2Port() {
        return drive2.getPortNumber();
    }
    /**
     * get port nuber
     */
    public int getDrive3Port() {
        return drive3.getPortNumber();
    }
    /**
     * get port nuber
     */
    public int getDrive4Port() {
        return drive4.getPortNumber();
    }
}
