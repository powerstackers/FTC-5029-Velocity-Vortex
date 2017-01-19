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

package com.powerstackers.velocity.opmodes.teleop;

import com.powerstackers.velocity.common.VelRobot;
import com.powerstackers.velocity.common.enums.PublicEnums.GrabberSetting;
import com.powerstackers.velocity.common.enums.PublicEnums.MotorSetting;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * @author Derek Helm
 */

@SuppressWarnings("unused")
@TeleOp(name="VEL-Teleop", group ="Powerstackers")
public class VelTeleop extends OpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private VelRobot robot;

    private boolean flag_grabberBeenReleased = false;
    private boolean flag_shootButtonJustPressed = false;
    private boolean flag_shooterIsOn = false;

    @Override
    public void init() {
        //init code is in main VelRobot class
        robot = new VelRobot(this);
        robot.initializeRobot(); //is this a thing?
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addLine("Hi! I'm working!");
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

        telemetry.addData("Shooter RPM", robot.getShooterRPM());
        telemetry.addData("shooterEncVal", robot.getShooterEncVal());
        telemetry.addData("Status", "Running: ");

        robot.setShooter(MotorSetting.FORWARD);
        //DbgLog.msg("RPM-- " + robot.getShooterRPM());

        // Read the joystick and determine what motor setting to use.

        //button maps here vvv
        boolean buttonVexMotorForward  = gamepad1.dpad_up;
        boolean buttonVexMotorBackward = gamepad1.dpad_down;
        boolean buttonParticlePickupIn = gamepad2.left_bumper;
        boolean buttonParticlePickupOut = gamepad2.left_trigger > 0.5;
        boolean buttonShooter = gamepad2.a;
        boolean buttonLiftUp = gamepad2.right_bumper;
        boolean buttonLiftDown = gamepad2.right_trigger > 0.5;
        boolean buttonGrabberRelease = gamepad2.b;
        boolean buttonBallSqueeze = gamepad2.x;

        // Set the movement of the robot's wheels
        robot.setMovement(VelRobot.mecDirectionFromJoystick(gamepad1),
                VelRobot.mecSpeedFromJoystick(gamepad1), VelRobot.mecSpinFromJoystick(gamepad1));

        // Set particle pickup motor
        if (buttonParticlePickupIn) {
            robot.setBallPickup(MotorSetting.REVERSE);
        } else if (buttonParticlePickupOut) {
            robot.setBallPickup(MotorSetting.FORWARD);
        } else {
            robot.setBallPickup(MotorSetting.STOP);
        }

        // Toggle the shooter on every press of the A button
        /*
        if (buttonShooter && !flag_shootButtonJustPressed) { ~richie did this
            flag_shootButtonJustPressed = true;
            flag_shooterIsOn = !flag_shooterIsOn;
        } else if (!buttonShooter) {
            flag_shootButtonJustPressed = false;
        }

        if (flag_shooterIsOn) {
            robot.rampShooter();
        } else {
            robot.setShooter(MotorSetting.STOP);
        }
        */

        // Set the Shootor motor value.
        if (buttonShooter) {
            robot.setShooter(MotorSetting.FORWARD);
        } else {
            robot.setShooter(MotorSetting.STOP);
        }

        // Set lift motor
        if (buttonLiftUp) {
            robot.setLift(MotorSetting.FORWARD);
        } else if (buttonLiftDown){
            robot.setLift(MotorSetting.REVERSE);
        } else {
            robot.setLift(MotorSetting.STOP);
        }

        // Only move the ball grabber after it has been deployed
        if (flag_grabberBeenReleased) {
            robot.setBallGrab(buttonBallSqueeze ? GrabberSetting.TIGHT : GrabberSetting.LOOSE);
        }

        // Release the ball grabber
        if (buttonGrabberRelease) {
            robot.releaseBallGrab();
            flag_grabberBeenReleased = true;
        }

//        telemetry here vvv
        telemetry.addData("left x", gamepad1.left_stick_x);
        telemetry.addData("left y", gamepad1.left_stick_y);
        telemetry.addData("right x", gamepad1.right_stick_x);
        telemetry.addData("rotation", VelRobot.mecSpinFromJoystick(gamepad1));
        telemetry.addData("shooting Power: ", robot.getShooterPower());
        telemetry.addData("Encoder: ", robot.getEncoderShooter());
//        telemetry.addData("color Alpha: ", robot.getAlpha());
//        telemetry.addData("color Red: ", robot.getRed());
//        telemetry.addData("color Green: ", robot.getGreen());
//        telemetry.addData("color Blue: ", robot.getGreen());
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