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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Basic configurations for our robot. This class contains methods to make the robot do stuff.
 *
 * @author Cate Thomas
 */

public class VelRobot {

    OpMode mode;
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

//    public CRServo vexMotor;


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
        stopMovement();
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
        // TODO Minimum speed threshold?

        double multipliers[] = new double[4];
        multipliers[0] = (speed * Math.sin(angle + (Math.PI/4))) + rotation;
        multipliers[1] = (speed * Math.cos(angle + (Math.PI/4))) - rotation;
        multipliers[2] = (speed * Math.cos(angle + (Math.PI/4))) + rotation;
        multipliers[3] = (speed * Math.sin(angle + (Math.PI/4))) - rotation;

        double largest = multipliers[0];
        for (int i = 1; i < 4; i++) {
            if (multipliers[i] > largest)
                largest = multipliers[i];
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
     * set vexmotor power
     */
//    public void vexPower(double power) {
//        vexMotor.setPower(power);
//    }

    /**
     *  Completely stop the drive motors.
     */
    public void stopMovement() {
        drive1.setPower(0.0);
        drive2.setPower(0.0);
        drive3.setPower(0.0);
        drive4.setPower(0.0);

//        vexMotor.setPower(0);
    }

    /**
     * Allows robot to go all the way froward and backwards on the y-axis
     *
     * @param pad Gamepad to take control values from.
     * @return A directiovoidn of movement, in radians, where "forward" is pi/2
     */
    public static double mecDirection(Gamepad pad) {
        double x = pad.left_stick_x;
        double y = -pad.left_stick_y;   // The Y stick is inverted

        // If x is exactly 0, atan will be undefined. In that case, our angle is either 90 or 270.
        if (x == 0) {
            return ((y > 0)? Math.PI / 2 : (Math.PI * 3) / 2);
        } else {
            double atan = Math.atan(y / x);

            // Make sure the angle is in the right quadrant.
            if (x > 0) {
                return ((y > 0)? atan : atan + (Math.PI * 2));
            } else {
                return atan + Math.PI;
            }
        }
    }

    /**
     *  Get the translation speed value from the joystick.
     *
     * @param pad Gamepad to take control values from.
     * @return Speed ranging from 0:1
     */
    public static double mecSpeed(Gamepad pad) {
        return Math.sqrt((pad.left_stick_y * pad.left_stick_y)
                + (pad.left_stick_x * pad.left_stick_x));
    }

    /**
     *  Get the spin speed value from the joystick.
     *
     * @param pad Gamepad to take control values from.
     * @return Speed ranging from -1:1
     */
    public static double mecSpin(Gamepad pad) {
        return (double) pad.right_stick_x;
    }

    /**
     * get VexMotor power
     */
//    public double getVexPower() {
//        return vexMotor.getPower();
//    }
}
