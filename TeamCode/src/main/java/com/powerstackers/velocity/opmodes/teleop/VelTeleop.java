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

import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.ftcrobotcontroller.R;

import com.powerstackers.velocity.common.VelRobot;
import com.powerstackers.velocity.common.enums.PublicEnums.MotorSetting;
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

    private boolean flag_speedToggleJustPressed = false;
    private boolean flag_speedChanged = false;
    private double scale = 0.0;

//    final View relativeLayout = ((Activity) robot.getOpMode().hardwareMap.appContext).findViewById(R.id.RelativeLayout);

    @Override
    public void init() {
        robot = new VelRobot(this);
        robot.initializeRobot();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addLine("Waiting for start...");
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

        //button maps here vvv
        boolean buttonParticlePickupIn  = gamepad2.left_bumper;
        boolean buttonParticlePickupOut = gamepad2.left_trigger > 0.5;
        boolean buttonShooter           = gamepad2.a;
        boolean buttonLiftUp            = gamepad2.right_bumper;
        boolean buttonLiftDown          = gamepad2.right_trigger > 0.5;
        boolean buttonCapBallTighter    = gamepad2.dpad_down;
        boolean buttonCapBallLooser     = gamepad2.dpad_up;
        boolean buttonSpeedToggle       = gamepad1.a;

        // Toggle speed for driver
        if (buttonSpeedToggle && !flag_speedToggleJustPressed) {
            flag_speedToggleJustPressed = true;
            flag_speedChanged = !flag_speedChanged;
        } else if (!buttonSpeedToggle) {
            flag_speedToggleJustPressed = false;
        }

        if (flag_speedChanged) {
            scale = 0.5;
        } else {
            scale = 1.0;
        }

        // Set the movement of the robot's wheels
        robot.setMovement(VelRobot.mecDirectionFromJoystick(gamepad1),
                          VelRobot.mecSpeedFromJoystick(gamepad1),
                          VelRobot.mecSpinFromJoystick(gamepad1),
                          scale);

        // Set particle pickup motor
        if (buttonParticlePickupIn) {
            robot.setBallPickup(MotorSetting.REVERSE);
        } else if (buttonParticlePickupOut) {
            robot.setBallPickup(MotorSetting.FORWARD);
        } else {
            robot.setBallPickup(MotorSetting.STOP);
        }

        // Set the Shootor motor value.
        // TODO Make shooter able to spin backwards for emergencies
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

        // Set cap ball grabber
        if (buttonCapBallLooser) {
            robot.servoBallGrab.setPosition(robot.servoBallGrab.getPosition() + 0.2);
        } else if (buttonCapBallTighter) {
            robot.servoBallGrab.setPosition(robot.servoBallGrab.getPosition() - 0.05);
        }

        // Screen will turn green if within target RPM, else screen is red
        // Not tested!
        // Why is VelRobotConstants not public??
//        if(robot.getShooterRPM() > (950 - 50) == robot.getShooterRPM() < (950 + 50)) {
//            relativeLayout.post(new Runnable() {
//                public void run() {
//                    relativeLayout.setBackgroundColor(Color.GREEN);
//                }
//            });
//        } else {
//            relativeLayout.post(new Runnable() {
//                public void run() {
//                    relativeLayout.setBackgroundColor(Color.RED);
//                }
//            });
//        }

//        telemetry here vvv
        telemetry.addData("RPM", robot.getShooterRPM());
        telemetry.addData("EncVal", robot.getShooterEncVal());
        telemetry.addData("Clear", robot.getAlpha());
        telemetry.addData("Red  ", robot.getRed());
        telemetry.addData("Green", robot.getGreen());
        telemetry.addData("Blue ", robot.getBlue());
    }

    /**
     * Stop the robot and make any final assignments.
     */
    @Override
    public void stop() {
        robot.stopAllMotors();
    }
}