package com.powerstackers.velocity.opmodes.teleop;

import com.powerstackers.velocity.common.VelRobot;
import com.powerstackers.velocity.common.enums.PublicEnums;
import com.powerstackers.velocity.common.enums.PublicEnums.AllianceColor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * @author Derek Helm
 */

@TeleOp(name="VEL-Teleop", group ="Powerstackers")
public class VelTeleop extends OpMode {

    //constructors
    AllianceColor allianceColor;
    VelRobot robot;

    //declarations here vvv
    boolean buttonVexMotorForward;
    boolean buttonVexMotorBackward;


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
        buttonVexMotorForward  = gamepad1.dpad_up;
        buttonVexMotorBackward = gamepad1.dpad_down;

        //if else statements here vvv
        if (buttonVexMotorForward) {
            robot.vexPower(1);
        } else if (buttonVexMotorBackward) {
            robot.vexPower(-1);
        } else {
            robot.vexPower(0);
        }

        //telemetry here vvv
        telemetry.addData("VexMotor : ", robot.getVexPower());

    }

    /**
     * Stop the robot and make any final assignments.
     */
    @Override
    public void stop() {
        //stop code here vvv

    }
}
