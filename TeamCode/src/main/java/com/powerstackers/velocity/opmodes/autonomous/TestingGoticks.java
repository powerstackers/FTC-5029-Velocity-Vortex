package com.powerstackers.velocity.opmodes.autonomous;

import com.powerstackers.velocity.common.VelRobotAuto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * @author Jonathan
 */
@Autonomous
public class TestingGoticks extends LinearOpMode {

    VelRobotAuto robot = new VelRobotAuto(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initializeRobot();
        waitForStart();
        robot.goTicks(robot.inchesToTicks(18), 1.0);
    }
}
