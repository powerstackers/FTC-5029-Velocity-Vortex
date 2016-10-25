package com.powerstackers.velocity.opmodes.teleop;

import com.powerstackers.velocity.common.VelRobot;
import com.powerstackers.velocity.common.enums.PublicEnums.AllianceColor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * @author Derek Helm
 */

@TeleOp(name="VEL-Teleop", group ="Powerstackers")
public class VelTeleop extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //constructors
    AllianceColor allianceColor;
    VelRobot robot;

    //declarations here vvv
//    boolean buttonVexMotorForward;
//    boolean buttonVexMotorBackward;


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
        robot.initializeRobot(); //is this a thing?


//        robot.vexMotor = hardwareMap.crservo.get("vexMotor");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }


    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Running: ");

        //button maps here vvv
//        buttonVexMotorForward  = gamepad1.dpad_up;
//        buttonVexMotorBackward = gamepad1.dpad_down;

        //if else statements here vvv
//        if (buttonVexMotorForward) {
//            robot.vexPower(1);
//        } else if (buttonVexMotorBackward) {
//            robot.vexPower(-1);
//        } else {
//            robot.vexPower(0);
//        }

        //telemetry here vvv
//        telemetry.addData("VexMotor : ", robot.getVexPower());

    }

    /**
     * Stop the robot and make any final assignments.
     */
    @Override
    public void stop() {
        //stop code here vvv
        robot.stopMovement();

    }
}
