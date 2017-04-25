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

/**
 * @author Derek Helm
 */
@Autonomous(name = "AutoTest", group = "Powerstackers")

public class VelAutonomousProgramTesting extends LinearOpMode {
    VelRobotAuto robot;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new VelRobotAuto(this);

        // Initialize any sensors and servos
        robot.initializeRobot();

        // Wait for the start of the match!Thread.interrupted()
        this.waitForStart();

        robot.driveWithUS(VelRobotConstants.DIRECTION_SOUTHEAST, 0.4, 20);
        robot.driveToLine(VelRobotConstants.DIRECTION_SOUTH, 0.2, PublicEnums.GyroCorrection.NO, PublicEnums.BeaconNumber.TWO, 0.8);
//        robot.setMovement(VelRobotConstants.DIRECTION_SOUTH, 0.2, 0, 1);
        Thread.sleep(200);
        robot.driveToLine(VelRobotConstants.DIRECTION_NORTH, 0.2, PublicEnums.GyroCorrection.NO, PublicEnums.BeaconNumber.TWO, 0.8);
        robot.driveWithUS(VelRobotConstants.DIRECTION_EAST, 0.2, 6);
        robot.beaconTap(PublicEnums.AllianceColor.BLUE);
        robot.setMovement(VelRobotConstants.DIRECTION_NORTH, 0.2,0, 0.8);
        Thread.sleep(200);
        robot.driveToLine(VelRobotConstants.DIRECTION_NORTH, 0.2, PublicEnums.GyroCorrection.NO, PublicEnums.BeaconNumber.TWO, 0.8);
        Thread.sleep(200);
        robot.driveToLine(VelRobotConstants.DIRECTION_SOUTH, 0.2, PublicEnums.GyroCorrection.NO, PublicEnums.BeaconNumber.TWO, 0.8);
        robot.driveWithUS(VelRobotConstants.DIRECTION_EAST, 0.2, 6);
        robot.beaconTap(PublicEnums.AllianceColor.BLUE);


    }
}
