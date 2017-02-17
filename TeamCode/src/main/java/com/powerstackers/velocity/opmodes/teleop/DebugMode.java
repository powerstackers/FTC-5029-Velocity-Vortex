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
import com.powerstackers.velocity.common.enums.PublicEnums;
import com.powerstackers.velocity.common.enums.PublicEnums.MotorSetting;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * @author Derek Helm
 */

@SuppressWarnings("unused")
@TeleOp(name="DebugMode", group ="Powerstackers")
public class DebugMode extends OpMode {

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
        boolean buttonTapBeacon         = gamepad1.y;

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

        //set tap beacon
//        if(buttonTapBeacon) {
//            robot.tapBeacon(PublicEnums.AllianceColor.RED);
//        } else {
//
//        }

        //ColorSensor Controls
        robot.sensorColor.enableLed(false);
        robot.sensorColorGroundL.enableLed(true);
        robot.sensorColorGroundR.enableLed(true);

        if (robot.sensorColor.blue() > robot.sensorColor.red()) {
//            servoBeaconPosition = 0.20;
            robot.setBeaconTap(0.20);

        } else if (robot.sensorColor.red() > robot.sensorColor.blue()) {
//            servoBeaconPosition = 0.80;
            robot.setBeaconTap(0.80);
        } else {
//            servoBeaconPosition = 0.50;
            robot.setBeaconTap(0.50);
        }

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
        telemetry.addData("Shooter RPM", robot.getShooterRPM());
        telemetry.addData("Front Left", robot.getDrive1Encoder());
        telemetry.addData("Front Right", robot.getDrive2Encoder());
        telemetry.addData("Back Left", robot.getDrive3Encoder());
        telemetry.addData("Back Right", robot.getDrive4Encoder());
//        telemetry.addData("EncVal", robot.getShooterEncVal());
//        telemetry.addData("Clear", robot.getAlpha());
//        telemetry.addData("Red  ", robot.getRed());
//        telemetry.addData("Green", robot.getGreen());
//        telemetry.addData("Blue ", robot.getBlue());
        telemetry.addData("L-Clear", robot.sensorColorGroundL.alpha());
        telemetry.addData("L-Red  ", robot.sensorColorGroundL.red());
        telemetry.addData("L-Green", robot.sensorColorGroundL.green());
        telemetry.addData("L-Blue ", robot.sensorColorGroundL.blue());
        telemetry.addData("R-Clear", robot.sensorColorGroundR.alpha());
        telemetry.addData("R-Red  ", robot.sensorColorGroundR.red());
        telemetry.addData("R-Green", robot.sensorColorGroundR.green());
        telemetry.addData("R-Blue ", robot.sensorColorGroundR.blue());
    }

    /**
     * Stop the robot and make any final assignments.
     */
    @Override
    public void stop() {
        robot.stopAllMotors();
    }
}