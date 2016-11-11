package com.powerstackers.velocity.common;

import com.powerstackers.velocity.common.enums.PublicEnums;
import com.powerstackers.velocity.common.enums.StartingPosition;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static com.powerstackers.velocity.common.enums.PublicEnums.AllianceColor.BLUE;
import static com.powerstackers.velocity.common.enums.PublicEnums.AllianceColor.RED;

/**
 * @author Derek Helm
 */

public class VelAutonomousProgram extends LinearOpMode {

    PublicEnums.AllianceColor allianceColor;
    StartingPosition startingPosition;
    VelRobotAuto robot;

    public VelAutonomousProgram(PublicEnums.AllianceColor allianceColor,
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

        if (allianceColor == RED) {

            //red autonomous code here vvv

        } else if (allianceColor == BLUE) {

            //blue autonomous code here vvv

        } else {
            telemetry.addData("choosered", "deprecated: ");
            stop();
        }
    }
}
