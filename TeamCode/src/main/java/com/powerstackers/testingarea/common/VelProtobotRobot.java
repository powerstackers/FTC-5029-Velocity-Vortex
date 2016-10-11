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

package com.powerstackers.testingarea.common;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * @author Derek Helm
 */

public class VelProtobotRobot {


    /*
    Looking at the robot from above:
        -------------
        |1         2|
        |           |
        |           |
        |3         4|
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
    public VelProtobotRobot(OpMode mode) {
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
    }

    
}
