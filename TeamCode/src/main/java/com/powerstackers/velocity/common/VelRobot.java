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

/**
 * @author Cate Thomas
 */

public class VelRobot {


    /*
    Looking at the robot from above:
        -------------
        |1\\     //2|
        |           |
        |           |
        |3//     \\4|
        -------------
     */
    private DcMotor drive1;
    private DcMotor drive2;
    private DcMotor drive3;
    private DcMotor drive4;


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

    /**
     * Set the movement speeds of all four motors, based on a desired angle, speed, and rotation
     * speed.
     * DEREK! NO TOUCH!
     * @param angle
     * @param speed
     * @param rotation
     */
    public void setMovement(double angle, double speed, double rotation) {
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

    public void stopMovement() {
        drive1.setPower(0.0);
        drive2.setPower(0.0);
        drive3.setPower(0.0);
        drive4.setPower(0.0);
    }
}
