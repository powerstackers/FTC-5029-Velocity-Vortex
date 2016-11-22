/*
 * Copyright (C) 2016 Powerstackers
 *
 * Teleop code for Velocity Vortex.
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

import com.powerstackers.velocity.common.enums.PublicEnums.MotorSetting;
import com.powerstackers.velocity.common.enums.PublicEnums.AllianceColor;
import com.powerstackers.velocity.common.enums.StartingPosition;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static com.powerstackers.velocity.common.enums.PublicEnums.AllianceColor.BLUE;
import static com.powerstackers.velocity.common.enums.PublicEnums.AllianceColor.RED;

/**
 * @author Derek Helm
 */
public class VelAutonomousProgram extends LinearOpMode {

    AllianceColor allianceColor;
    StartingPosition startingPosition;
    VelRobotAuto robot;

    public VelAutonomousProgram(AllianceColor allianceColor,
                                StartingPosition startingPosition) {
        this.allianceColor = allianceColor;
        this.startingPosition = startingPosition;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize any sensors and servos
        robot = new VelRobotAuto(this);
        robot.initializeRobot();
        // Wait for the start of the match!Thread.interrupted()
        this.waitForStart();

        // Turn on the shooters now, to give them time to spin up
        robot.setShooter(MotorSetting.FORWARD);

        // Move into position
        robot.goDistanceInCm(65.0, VelRobotConstants.DIRECTION_NORTH, 1.0);

        // Feed balls into shooter
        robot.setBallPickup(MotorSetting.FORWARD);

        // Wait for balls to shoot
        sleep(3000);

        // Stop shooter
        robot.setBallPickup(MotorSetting.STOP);
        robot.setShooter(MotorSetting.STOP);

        // Move into ball
        robot.goDistanceInCm(65.0, VelRobotConstants.DIRECTION_NORTH, 1.0);
    }
}
