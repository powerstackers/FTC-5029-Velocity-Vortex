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

import static com.powerstackers.velocity.common.enums.PublicEnums.AllianceColor.BLUE;
import static com.powerstackers.velocity.common.enums.PublicEnums.AllianceColor.RED;

/**
 * @author Derek Helm
 */
@Autonomous(name = "AutoTest", group = "Powerstackers")

public class VelAutonomousProgramTesting extends LinearOpMode {
    VelRobotAuto robot;


    @Override
    public void runOpMode() throws InterruptedException {
        //telemetry.addData("Robot: ", "robot starting init");
        telemetry.update();
        robot = new VelRobotAuto(this);

        // Initialize any sensors and servos
        robot.initializeRobot();
        //telemetry.addData("Robot: ", "robot init");
        //telemetry.update();
        // Wait for the start of the match!Thread.interrupted()
        this.waitForStart();
//        telemetry.addData("Is there mat: ", robot.isThereMat());
//        telemetry.update();
            robot.encoderDriveCm(VelRobotConstants.DIRECTION_NORTH, 0.8, 30);
        //robot.driveWithUS(VelRobotConstants.DIRECTION_SOUTH, 0.5, 10);
//        robot.turnDegreesRight(VelRobotConstants.DIRECTION_EAST, 0.4);
        //robot.turnDegreesLeft(90, 0.4);
//
//        robot.driveToLine(VelRobotConstants.DIRECTION_SOUTHEAST, 0.8, PublicEnums.GyroCorrection.NO, PublicEnums.BeaconNumber.ONE, 1);
//        robot.driveWithUS(VelRobotConstants.DIRECTION_EAST, 0.5, 5);
//        robot.driveToLine(VelRobotConstants.DIRECTION_NORTH, 0.4, PublicEnums.GyroCorrection.NO, PublicEnums.BeaconNumber.ONE, 1);
//        robot.beaconTap(RED);

    }
}
