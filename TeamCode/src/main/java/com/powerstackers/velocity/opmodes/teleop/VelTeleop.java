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

import com.powerstackers.velocity.common.VelRobot;
import com.powerstackers.velocity.common.VelRobotConstants;
import com.powerstackers.velocity.common.enums.PublicEnums;
import com.powerstackers.velocity.common.enums.PublicEnums.MotorSetting;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.R;

import static com.powerstackers.velocity.common.VelRobotConstants.MINIMUM_JOYSTICK_THRESHOLD;

/**
 * @author Derek Helm
 */

@SuppressWarnings("unused")
@TeleOp(name = "VEL-Teleop", group = "Powerstackers")
public class VelTeleop extends OpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private VelRobot robot;

    private boolean flag_grabberBeenReleased = false;
    private boolean flag_shootButtonJustPressed = false;
    private boolean flag_shooterIsOn = false;

    private boolean flag_speedToggleJustPressed = false;
    private boolean flag_speedChanged = false;
    private double scale = VelRobotConstants.DRIVE_SPEED_NORMAL; // Normal Speed Needs to be tested
    private double rotationNegation = 1;
    //final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);

    @Override
    public void init() {

        robot = new VelRobot(this);
        try {
            robot.initializeRobot();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
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
        robot.displayDirection();
        //button maps here vvv
        boolean buttonParticlePickupIn = gamepad2.left_bumper;
        boolean buttonParticlePickupOut = gamepad2.left_trigger > 0.5;
        boolean buttonShooter = gamepad2.a;
        boolean buttonLiftUp = gamepad2.right_bumper;
        boolean buttonLiftDown = gamepad2.right_trigger > 0.5;
        boolean buttonCapBallTighter = gamepad2.dpad_down;
        boolean buttonCapBallLooser = gamepad2.dpad_up;
        boolean buttonSpeedFastToggle = gamepad1.right_bumper;
        boolean buttonSpeedSlowHold = gamepad1.left_bumper;
        boolean buttonTapBeacon = gamepad1.y;
        boolean buttonShoot = gamepad2.b;
        boolean buttonBeaconRight = gamepad1.right_trigger > 0.5;
        boolean buttonBeaconLeft = gamepad1.left_trigger > 0.5;

        // Toggle speed for driver
        if (buttonSpeedFastToggle && !flag_speedToggleJustPressed) {
            flag_speedToggleJustPressed = true;
            flag_speedChanged = !flag_speedChanged;
        } else if (!buttonSpeedFastToggle) {
            flag_speedToggleJustPressed = false;
        }

        if (flag_speedChanged) {
            scale = VelRobotConstants.DRIVE_SPEED_FAST;
            telemetry.addData("Drive Speed: ", "Fast");
            telemetry.update();
        } else {

            scale = VelRobotConstants.DRIVE_SPEED_NORMAL;
            telemetry.addData("Drive Speed: ", "Normal");
            telemetry.update();
        }

        // Set the movement of the robot's wheels
        if (robot.robotDirection == PublicEnums.Direction.E || robot.robotDirection == PublicEnums.Direction.W) {
            rotationNegation = -1;
        } else {
            rotationNegation = 1;
        }
        if (buttonSpeedSlowHold) {
            scale = VelRobotConstants.DRIVE_SPEED_SLOW;
            robot.setMovement(VelRobot.mecDirectionFromJoystick(gamepad1),
                    VelRobot.mecSpeedFromJoystick(gamepad1),
                    VelRobot.mecSpinFromJoystick(gamepad1) * 2 * rotationNegation,
                    scale);
            telemetry.addData("Drive Speed: ", "Slow");
            telemetry.update();
        } else {
            robot.setMovement(VelRobot.mecDirectionFromJoystick(gamepad1),
                    VelRobot.mecSpeedFromJoystick(gamepad1),
                    VelRobot.mecSpinFromJoystick(gamepad1) * 2 * rotationNegation,
                    scale);
            telemetry.addData("Drive Speed: ", "Normal");
            telemetry.update();
        }
        if (gamepad1.dpad_up) {
            robot.directionChange(PublicEnums.Direction.N);
        }
        if (gamepad1.dpad_left) {
            robot.directionChange(PublicEnums.Direction.W);
        }
        if (gamepad1.dpad_down) {
            robot.directionChange(PublicEnums.Direction.S);
        }
        if (gamepad1.dpad_right) {
            robot.directionChange(PublicEnums.Direction.E);
        }

        //set tap beacon
//        if(buttonTapBeacon) {
//            robot.tapBeacon(PublicEnums.AllianceColor.RED);
//        } else {
//
//        }

        //ColorSensor Controls
        robot.sensorColor.enableLed(false);
//        robot.sensorColorGroundL.enableLed(true);
//        robot.sensorColorGroundR.enableLed(true);


        // Set particle pickup motor
        if (buttonParticlePickupIn) {
            robot.setBallPickup(MotorSetting.REVERSE);
        } else if (buttonParticlePickupOut) {
            robot.setBallPickup(MotorSetting.FORWARD);
        } else {
            robot.setBallPickup(MotorSetting.STOP);
        }
        if (buttonBeaconLeft) {
            robot.leftBeaconPosition(PublicEnums.BeaconPostion.IN);
        }else{
            robot.leftBeaconPosition(PublicEnums.BeaconPostion.OUT);
        }
        if (buttonBeaconRight) {
            robot.rightBeaconPosition(PublicEnums.BeaconPostion.IN);
        }else{
            robot.rightBeaconPosition(PublicEnums.BeaconPostion.OUT);
        }
        // Set the Shooter motor value.
        // TODO Make shooter able to spin backwards for emergencies
        if (buttonShooter) {
            robot.setShooter(MotorSetting.FORWARD);
        } else {
            robot.setShooter(MotorSetting.STOP);
        }

        // Set lift motor
        if (buttonLiftUp) {
            robot.setLift(MotorSetting.FORWARD);
        } else if (buttonLiftDown) {
            robot.setLift(MotorSetting.REVERSE);
        } else {
            robot.setLift(MotorSetting.STOP);
        }

        // Set cap ball grabber
        if (buttonCapBallLooser) {
            robot.servoBallGrab.setPosition(1);
        } else if (buttonCapBallTighter) {
            robot.servoBallGrab.setPosition(0);
        }
        else{
            robot.servoBallGrab.setPosition(0.5);
        }
        if (buttonShoot) {
            robot.servoShoot.setPosition(VelRobotConstants.SHOOT_SERVO_OPEN);
        } else {
            robot.servoShoot.setPosition(VelRobotConstants.SHOOT_SERVO_CLOSED);
        }

//        robot.motorShooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.motorShooter1.setPower(1);
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
//        if (robot.isShooterRunning()) {
//            telemetry.addData("Shooter RPM", robot.getShooterRPM());
//            relativeLayout.post(new Runnable() {
//                public void run() {
//                    if (robot.getShooterRPM() < (VelRobotConstants.MOTOR_SHOOTER_TARGET_RPM + 70) && robot.getShooterRPM() > (VelRobotConstants.MOTOR_SHOOTER_TARGET_RPM - 70)) {
//                        relativeLayout.setBackgroundColor(Color.GREEN);
//                    } else {
//                        relativeLayout.setBackgroundColor(Color.RED);
//                    }
//                }
//            });
        }
//        telemetry.addData("EncVal", robot.getShooterEncVal());
//        telemetry.addData("Clear", robot.getAlpha());
//        telemetry.addData("Red  ", robot.getRed());
//        telemetry.addData("Green", robot.getGreen());
//        telemetry.addData("Blue ", robot.getBlue());
//        telemetry.addData("L-Clear", robot.sensorColorGroundL.alpha());
//        telemetry.addData("L-Red  ", robot.sensorColorGroundL.red());
//        telemetry.addData("L-Green", robot.sensorColorGroundL.green());
//        telemetry.addData("L-Blue ", robot.sensorColorGroundL.blue());
//        telemetry.addData("R-Clear", robot.sensorColorGroundR.alpha());
//        telemetry.addData("R-Red  ", robot.sensorColorGroundR.red());
//        telemetry.addData("R-Green", robot.sensorColorGroundR.green());
//        telemetry.addData("R-Blue ", robot.sensorColorGroundR.blue());
    }

    /**
     * Stop the robot and make any final assignments.
     */
    @Override
    public void stop() {
        robot.stopAllMotors();
    }
}