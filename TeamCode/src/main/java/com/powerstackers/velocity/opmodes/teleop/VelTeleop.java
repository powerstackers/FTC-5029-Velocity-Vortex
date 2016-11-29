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
import com.powerstackers.velocity.common.enums.PublicEnums.AllianceColor;
import com.powerstackers.velocity.common.enums.PublicEnums.GrabberSetting;
import com.powerstackers.velocity.common.enums.PublicEnums.MotorSetting;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * @author Derek Helm
 */

@TeleOp(name="VEL-Teleop", group ="Powerstackers")
public class VelTeleop extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private static final float MINIMUM_JOYSTICK_THRESHOLD = 0.15F;

    //constructors
    AllianceColor allianceColor;
    VelRobot robot;


    boolean buttonVexMotorForward;
    boolean buttonVexMotorBackward;
    boolean buttonParticlePickupIn;
    boolean buttonParticlePickupOut;
    boolean buttonShooter;
    boolean buttonLiftUp;
    boolean buttonLiftDown;
    boolean buttonGrabberRelease;
    boolean buttonBallSqueeze;

    boolean flag_grabberBeenReleased = false;

    /**
    * Default constructor. Need this!!!
    * @return
    */
    public VelTeleop() {

    }

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
        telemetry.addData("Status", "Running: ");

        // Read the joystick and determine what motor setting to use.

        //button maps here vvv
        buttonVexMotorForward  = gamepad1.dpad_up;
        buttonVexMotorBackward = gamepad1.dpad_down;
        buttonParticlePickupIn = gamepad2.left_bumper;
        buttonParticlePickupOut = gamepad2.left_trigger > 0.5;
        buttonShooter = gamepad2.a;
        buttonLiftUp = gamepad2.right_bumper;
        buttonLiftDown = gamepad2.right_trigger > 0.5;
        buttonGrabberRelease = gamepad2.b;
        buttonBallSqueeze = gamepad2.x;

//        if else statements here vvv
        if (buttonVexMotorForward) {
            robot.vexPower(1);
        } else if (buttonVexMotorBackward) {
            robot.vexPower(-1);
        } else {
            robot.vexPower(0);
        }

        // Set the movement of the robot's wheels
        robot.setMovement(VelRobot.mecDirectionFromJoystick(gamepad1),
                VelRobot.mecSpeedFromJoystick(gamepad1), VelRobot.mecSpinFromJoystick(gamepad1));

        // Set particle pickup motor
        robot.setBallPickup(buttonParticlePickupIn? MotorSetting.FORWARD :
                (buttonParticlePickupOut? MotorSetting.REVERSE : MotorSetting.STOP));

        // Set particle shooter
        robot.setShooter(buttonShooter? MotorSetting.FORWARD : MotorSetting.STOP);

        // Set lift motor
        robot.setLift(buttonLiftUp? MotorSetting.FORWARD :
                (buttonLiftDown? MotorSetting.REVERSE : MotorSetting.STOP));

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
        telemetry.addData("VexMotor : ", robot.getVexPower());
        telemetry.addData("Drive1 port:", robot.getDrive1Port());
        telemetry.addData("Power: ", robot.getDrive1Power());
        telemetry.addData("Drive2 port:", robot.getDrive2Port());
        telemetry.addData("Power: ", robot.getDrive2Power());
        telemetry.addData("Drive3 port:", robot.getDrive3Port());
        telemetry.addData("Power: ", robot.getDrive3Power());
        telemetry.addData("Drive4 port:", robot.getDrive4Port());
        telemetry.addData("Power: ", robot.getDrive4Power());

    }

    /**
     * Stop the robot and make any final assignments.
     */
    @Override
    public void stop() {
        //stop code here vvv
        robot.stopMovement();

    }

    /**
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }
}
