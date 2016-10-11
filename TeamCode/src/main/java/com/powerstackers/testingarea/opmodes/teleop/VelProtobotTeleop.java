package com.powerstackers.testingarea.opmodes.teleop;

import com.powerstackers.testingarea.common.VelProtobotRobot;
import com.powerstackers.velocity.common.enums.PublicEnums.AllianceColor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.abs;

/**
 * @author Derek Helm
 */

@TeleOp(name = "VEL-ProtoBotTeleop", group = "Powerstackers")
@Disabled
public class VelProtobotTeleop extends OpMode {

    private static final float MINIMUM_JOYSTICK_THRESHOLD = 0.15F;

    //constructors
    AllianceColor allianceColor;
    VelProtobotRobot robot;

    //declarations here vvv
    float stickDriveMoveY;
    float stickDriveMoveX;
    float stickDriveTurn;

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

        // Read the joystick and determine what motor setting to use.
        stickDriveMoveY = (float) scaleInput(Range.clip(-gamepad1.left_stick_y, -1, 1));
        stickDriveMoveX = (float) scaleInput(Range.clip(-gamepad1.left_stick_x, -1, 1));
        stickDriveTurn = (float) scaleInput(Range.clip(-gamepad1.right_stick_x, -1, 1));

        //if else statements here vvv

        // Last of all, update the motor values.
        // Move and Turn drive motors
        if ((abs(stickDriveMoveY) > MINIMUM_JOYSTICK_THRESHOLD) || (abs(stickDriveMoveX) > MINIMUM_JOYSTICK_THRESHOLD)) {
            robot.moveUpDown(-stickDriveMoveY);
            robot.moveLeftRight(-stickDriveMoveX);
        } else if (abs(stickDriveTurn) > MINIMUM_JOYSTICK_THRESHOLD) {
            robot.turnLeftRight(-stickDriveTurn);
        } else {
            robot.moveUpDown(0);
            robot.moveLeftRight(0);
            robot.turnLeftRight(0);
        }

        //telemetry here vvv

    }

    /**
     * Stop the robot and make any final assignments.
     */
    @Override
    public void stop() {
        //stop code here vvv

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
