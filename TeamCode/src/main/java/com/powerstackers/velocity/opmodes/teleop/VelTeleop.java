package com.powerstackers.velocity.opmodes.teleop;

import com.powerstackers.velocity.common.VelRobot;
import com.powerstackers.velocity.common.enums.PublicEnums.AllianceColor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.abs;

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

    //declarations here vvv
    float stickMove;
//    float stickMoveDown;
//    float stickMoveRight;
    float stickMoveLeftRight;
    float stickTurnRight;
    float stickTurnLeftRight;

    boolean buttonVexMotorForward;
    boolean buttonVexMotorBackward;

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
//        stickMoveUpDown    = (float) scaleInput(Range.clip(-gamepad1.left_stick_y, -1, 1));
//        stickMoveLeftRight  = (float) scaleInput(Range.clip(-gamepad1.left_stick_x, -1, 1));
//        stickTurnLeftRight  = (float) scaleInput(Range.clip(-gamepad1.right_stick_x, -1, 1));

//        stickMove    = (float) scaleInput(Range.clip(, -1, 1));
//        stickMoveLeftRight  = (float) scaleInput(Range.clip(-gamepad1.left_stick_x, -1, 1));
//        stickTurnLeftRight  = (float) scaleInput(Range.clip(robot.mecSpinFromJoystick(gamepad1), -1, 1));

        //button maps here vvv
        buttonVexMotorForward  = gamepad1.dpad_up;
        buttonVexMotorBackward = gamepad1.dpad_down;

//        if else statements here vvv
        if (buttonVexMotorForward) {
            robot.vexPower(1);
        } else if (buttonVexMotorBackward) {
            robot.vexPower(-1);
        } else {
            robot.vexPower(0);
        }

        // TODO: 10/27/16 add get stickTurnLeftRight Value -=left +=right

//        if (abs(stickMoveUpDown) > MINIMUM_JOYSTICK_THRESHOLD) {
//            robot.setMovement((PI/2), -stickMoveUpDown, 0);
//        } else if (abs(stickMoveLeftRight) > MINIMUM_JOYSTICK_THRESHOLD) {
//            if (stickMoveLeftRight < 0) {
//                robot.setMovement();
//            } else if (stickMoveLeftRight > 0) {
//                robot.setMovement(((3 * PI) / 2), -stickMoveLeftRight, 0);
//            }
//        } else if (abs(stickTurnLeftRight) > MINIMUM_JOYSTICK_THRESHOLD) {
//            robot.setMovement((PI/2), -stickTurnLeftRight, -stickTurnLeftRight);
//        } else {
//                robot.stopMovement();
//        }

        if (((abs(gamepad1.left_stick_y)) > MINIMUM_JOYSTICK_THRESHOLD) || ((abs(gamepad1.left_stick_x)) > MINIMUM_JOYSTICK_THRESHOLD) || ((abs(gamepad1.right_stick_x)) > MINIMUM_JOYSTICK_THRESHOLD)) {
            robot.setMovement(robot.mecDirectionFromJoystick(this.gamepad1), robot.mecSpeedFromJoystick(this.gamepad1), robot.mecSpinFromJoystick(this.gamepad1));
        } else {
            robot.stopMovement();
        }

//        if (abs(stickMove) > MINIMUM_JOYSTICK_THRESHOLD) {
//            robot.setMovement(robot.mecDirectionFromJoystick(gamepad1), stickMove, 0);
//        } else if (abs(stickMoveLeftRight) > MINIMUM_JOYSTICK_THRESHOLD) {
//            if (stickMoveLeftRight < 0) {
//                robot.setMovement(robot.mecDirectionFromJoystick(gamepad1), stickMoveLeftRight, 0);
//            } else if (stickMoveLeftRight > 0) {
//                robot.setMovement((2*PI), robot.mecSpeedFromJoystick(gamepad1), 0);
//            }
//        } else if (abs(stickTurnLeftRight) > MINIMUM_JOYSTICK_THRESHOLD) {
//            robot.setMovement(0, stickTurnLeftRight, robot.mecSpinFromJoystick(gamepad1));
//        } else {
//                robot.stopMovement();
//        }

//        telemetry here vvv
//        telemetry.addData("VexMotor : ", robot.getVexPower());
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
