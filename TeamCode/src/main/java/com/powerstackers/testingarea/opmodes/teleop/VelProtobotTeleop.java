package com.powerstackers.testingarea.opmodes.teleop;

import com.powerstackers.testingarea.common.VelProtobotRobot;
import com.powerstackers.velocity.common.enums.PublicEnums.AllianceColor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * @author Derek Helm
 */

public class VelProtobotTeleop extends OpMode {

    //constructors
    AllianceColor allianceColor;
    VelProtobotRobot robot;

    //declarations here vvv


    /**
     * Generate a new Teleop program with the given alliance color.
     * @param allianceColor The color that we are playing as this round.
     */
    public VelProtobotTeleop(AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
    }

    @Override
    public void init() {
        //init code is in main VelRobot class
        robot = new VelProtobotRobot(this);

    }

    @Override
    public void loop() {

        //button maps here vvv

        //if else statements here vvv

        //telemetry here vvv

    }

    /**
     * Stop the robot and make any final assignments.
     */
    @Override
    public void stop() {
        //stop code here vvv

    }
}
