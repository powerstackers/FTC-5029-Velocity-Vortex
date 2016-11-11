/*
 * Copyright (C) 2016 Powerstackers
 *
 * Code for autonomous.
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

package com.powerstackers.velocity.common;

import com.powerstackers.velocity.common.enums.PublicEnums;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.PI;
import static java.lang.Math.abs;

/**
 * Contains basic utility functions for autonomous control.
 * Things like turning, going in straight lines, and moving various manipulators can be controlled
 * from inside this class.
 *
 * @author Jonathan Thomas
 */
public class VelJonsAlgo {

    VelRobotAuto robot;
    OpMode mode;

    /**
     * Create a new VelJonsAlgo object. RobotAuto passed in must have a working parent opmode.
     * @param robot Robot object to use.
     */
    public VelJonsAlgo(VelRobotAuto robot) {
        this.robot = robot;
        this.mode = robot.getParentOpMode();
    }

    /*
    *	GLOBAL CONSTANTS
    */
    /**
     * Stores the number of encoder ticks in one motor revolution.
     * For AndyMark Neverest 40's, it's 280. The encoder tick count is actually 7 pulses per
     * revolution. Since the gearbox increases the number of rotations by a factor of 40, the final
     * count is 7 * 40 = 280. For 20 or 60 reduction motors, the  number would be different.
     */
    static double ticksPerRevolution = 1120; // Number of encoder ticks per motor rotation
    static double wheelDiameter = 4;         // Diameter of your wheels in inches
    static double driveGearMultiplier = 1.0; // Drive gear multiplier.
    // EXAMPLE: If your drive train is geared 2:1 (1 motor rotation = 2 wheel rotations), set this to 2
//    double turnOvershootThreshold = 0.1;

    /**
     * Converts a distance in inches to a distance in encoder ticks.
     * <p>We calculate this by taking the number of wheel rotations (inches/(PI*wheelDiameter))
     * multiplied by the inverse of the gear ratio, to get the number of motor rotations. Multiply
     * one more time by the number of motor encoder ticks per one motor revolution.
     * @param  inches double containing the distance you want to travel.
     * @return        that distance in encoder ticks.
     */
    public static double cmToTicks(double inches) {
        // TODO This is wrong now.
        return (long) ((1/driveGearMultiplier)*ticksPerRevolution*(inches/(PI*wheelDiameter)));
    }

    /**
     * Converts a distance in encoder ticks to a distance in inches.
     * <p>We calculate this by taking the number of ticks traveled, divided by the number of ticks
     * per revolution, and then multiplied by the gear ratio multiplier to get the number of wheel
     * rotations. Multiply one more time by the circumference of the wheels (PI*wheelDiameter).
     * @param  ticks long representing the distance in ticks.
     * @return       that distance in inches.
     */
    public static double ticksToCm(long ticks) {
        // TODO This is wrong.
        return (ticks/ticksPerRevolution)*driveGearMultiplier*(PI*wheelDiameter);
    }

    /**
     * Move the robot across the playing field a certain distance.
     * Indicating a negative speed or distance will cause the robot to move in reverse.
     * @param  distance The distance that we want to travel, in centimeters.
     * @param angle The angle to move at, in radians.
     * @param  speed The speed at which to travel.
     */
    public void goDistanceInCm(double distance, double angle, double speed) {
        // TODO Figure out how to do this.
    }

    /**
     * Turn the robot a certain number of degrees from center.
     * @param degrees Number of DEGREES to turn. Positive is counterclockwise, negative is clockwise.
     * @param speed Speed at which to turn.
     * @throws InterruptedException Make sure that we don't get trapped in this method when interrupted.
     */
    void turnDegrees(double degrees, double speed) throws InterruptedException {

        double degreesSoFar = robot.getGyroHeading();

        if (degrees > 180) {
//            robot.setPowerLeft(-1 * speed);
//            robot.setPowerRight(speed);
            mode.telemetry.addData("gyro1", robot.getGyroHeading());
        } else if (degrees < 180 || degrees == 180) {
//            robot.setPowerLeft(speed);
//            robot.setPowerRight(-1 * speed);
            mode.telemetry.addData("gyro2", robot.getGyroHeading());
        } else {
//            robot.setPowerAll(0);
        }
        mode.telemetry.addData("Gyro", degrees + "," + degreesSoFar);
        // For as long as the current degree measure doesn't equal the target. This will work in the clockwise and
        // counterclockwise directions, since we are comparing the absolute values
        while ((degreesSoFar) < (degrees)) {
            mode.telemetry.addData("gyrocompare", degreesSoFar=robot.getGyroHeading());
        }

        // Stop all drive motors
//        robot.setPowerAll(0);
    }

    /**
     * Tap the beacon on the correct side.
     * @param allianceColor The color that we are currently playing as.
     */
    public void tapBeacon(PublicEnums.AllianceColor allianceColor) {
        PublicEnums.AllianceColor dominantColor;
        double positionBeaconServo;

        // Detect the color shown on the beacon's left half, and record it.
        if (robot.sensorColor.red() > robot.sensorColor.blue()) {
            dominantColor = PublicEnums.AllianceColor.RED;
        } else {
            dominantColor = PublicEnums.AllianceColor.BLUE;
        }

        // Tap the correct side based on the dominant color.
        if (dominantColor == allianceColor) {
            positionBeaconServo = VelRobotConstants.BEACON_TAP_LEFT;
        } else {
            positionBeaconServo = VelRobotConstants.BEACON_TAP_RIGHT;
        }

        // Trim the servo value and set the servo position.
        positionBeaconServo = trimServoValue(positionBeaconServo);
        robot.servoBeacon.setPosition(positionBeaconServo);
    }

    /**
     * Trim a servo value between the minimum and maximum ranges.
     * @param servoValue Value to trim.
     * @return A raw double with the trimmed value.
     */
    private static double trimServoValue(double servoValue) {
        return Range.clip(servoValue, 0.0, 1.0);
    }
}
