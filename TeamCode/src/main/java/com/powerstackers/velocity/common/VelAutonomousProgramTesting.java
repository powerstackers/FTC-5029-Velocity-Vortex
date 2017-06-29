/*
 * Copyright (C) 2016 Powerstackers
 *
 * Autonomous code for Velocity Vortex.
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
import com.powerstackers.velocity.common.enums.PublicEnums.AllianceColor;
import com.powerstackers.velocity.common.enums.StartingPosition;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;

/**
 * @author Derek Helm
 */
@Autonomous(name = "AutoTest", group = "Powerstackers")

public class VelAutonomousProgramTesting extends LinearOpMode {
    VelRobotAuto robot = new VelRobotAuto(this);
    //Timer time = new Timer();

    @Override
    public void runOpMode() throws InterruptedException {

        //ElapsedTime runtime = new ElapsedTime();

        // Initialize any sensors and servos
        robot.initializeRobot();

        // Wait for the start of the match!Thread.interrupted()
        this.waitForStart();
        //runtime.reset();
        robot.setMovement(VelRobotConstants.DIRECTION_NORTH, 0.8, 0, 1);
        Thread.sleep(5000);
//        robot.servoShoot.setPosition(VelRobotConstants.SHOOT_SERVO_OPEN);
//        Thread.sleep(1000);
    }
}
