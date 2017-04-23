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

    final AllianceColor allianceColor;
    final StartingPosition startingPosition;
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

        if (allianceColor == BLUE && startingPosition == StartingPosition.FAR_FROM_RAMP) {

        } else if (allianceColor == BLUE && startingPosition == StartingPosition.MIDDLE) {
            robot.zeroEncoders();
            // TODO: make shooter shoot using new method
            robot.calibrateGyro();
            sleep(5000);
            robot.goTicks(robot.inchesToTicks(24),0.3);
            robot.setShooter(MotorSetting.FORWARD);
            sleep(1400);
//            robot.setBallPickup(MotorSetting.FORWARD);
//            sleep(1500);
//            robot.setBallPickup(MotorSetting.STOP);
            robot.setShooter(MotorSetting.STOP);

            sleep(1500);

            robot.setShooter(MotorSetting.FORWARD);
            sleep(1200);
//            robot.setBallPickup(MotorSetting.FORWARD);
//            sleep(2000);
//            robot.setShooter(MotorSetting.STOP);

            robot.goTicks(robot.inchesToTicks(26),0.3);

        } else if (allianceColor == RED && startingPosition == StartingPosition.FAR_FROM_RAMP) {

        } else if (allianceColor == RED && startingPosition == StartingPosition.MIDDLE) {
            robot.zeroEncoders();
            // TODO: make shooter shoot using new method
            robot.calibrateGyro();
            sleep(5000);
            robot.goTicks(robot.inchesToTicks(24),0.3);
            robot.setShooter(MotorSetting.FORWARD);
            sleep(1400);
//            robot.setBallPickup(MotorSetting.FORWARD);
//            sleep(1500);
//            robot.setBallPickup(MotorSetting.STOP);
//            robot.setShooter(MotorSetting.STOP);
//
//            sleep(1500);
//
//            robot.setShooter(MotorSetting.FORWARD);
//            sleep(1200);
//            robot.setBallPickup(MotorSetting.FORWARD);
//            sleep(2000);
            robot.setShooter(MotorSetting.STOP);

            robot.goTicks(robot.inchesToTicks(26),0.3);

        } else if (allianceColor == RED && startingPosition == StartingPosition.BACKUP) {

            robot.setPowerLeft(-0.3 * 0.75);
            robot.setPowerRight(-0.3 * 1.0);
            sleep(1800);
            robot.stopAllMotors();
            robot.setPowerLeft(+0.3 * 0.75);
            robot.setPowerRight(-0.3 * 1.0);
            sleep(500);
            robot.stopAllMotors();
            robot.setPowerLeft(-0.3 * 0.75);
            robot.setPowerRight(-0.3 * 1.0);
            sleep(1800);

        } else if (allianceColor == BLUE && startingPosition == StartingPosition.BACKUP) {

            robot.setPowerLeft(-0.3 * 0.75);
            robot.setPowerRight(-0.3 * 1.0);
            sleep(1800);
            robot.stopAllMotors();

        } else if (allianceColor == RED) {

        } else if (allianceColor == BLUE) {
//            robot.setPowerLeft(-0.3 * 0.75);
//            robot.setPowerRight(-0.3 * 1.0);
//            sleep(1800);
//            robot.stopAllMotors();

//            robot.calibrateGyro();
//            sleep(5000);
////            robot.goTicks(robot.inchesToTicks(23),0.8);
//            robot.setShooter(MotorSetting.FORWARD);
//            sleep(3000);
//            robot.setBallPickup(MotorSetting.FORWARD);
//            sleep(10000);
//            robot.goTicks(robot.inchesToTicks(35.5),0.8);

        } else {

        }
            //vvvvTEST?vvv
//        // Turn on the shooters now, to give them time to spin up
//        robot.setShooter(MotorSetting.FORWARD);
//
//        // Move into position
//        robot.goDistanceInCm(65.0, VelRobotConstants.DIRECTION_SOUTH, 1.0);
//
//        // Feed balls into shooter
//        robot.setBallPickup(MotorSetting.FORWARD);
//
//        // Wait for balls to shoot
//        sleep(3000);
//
//        // Stop shooter
//        robot.setBallPickup(MotorSetting.STOP);
//        robot.setShooter(MotorSetting.STOP);
//
//        // Move into ball
//        robot.goDistanceInCm(65.0, VelRobotConstants.DIRECTION_SOUTH, 1.0);
    }
}
