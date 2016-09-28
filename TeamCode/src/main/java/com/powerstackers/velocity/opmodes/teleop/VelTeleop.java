package com.powerstackers.velocity.opmodes.teleop;

import com.powerstackers.velocity.common.VelRobot;
import com.powerstackers.velocity.common.enums.PublicEnums;
import com.powerstackers.velocity.common.enums.PublicEnums.AllianceColor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by root on 9/25/16.
 */

public class VelTeleop extends OpMode {

    //constructors
    AllianceColor allianceColor;
    VelRobot robot;

    //declarations here vvv


    /**
     * Generate a new Teleop program with the given alliance color.
     * @param allianceColor The color that we are playing as this round.
     */
    public VelTeleop(AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
    }

    @Override
    public void init() {
        //init code is in main VelRobot class
        robot = new VelRobot(this);

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
