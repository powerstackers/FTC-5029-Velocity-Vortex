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

package com.powerstackers.velocity.common;

import com.powerstackers.velocity.common.enums.PublicEnums;
import com.powerstackers.velocity.common.enums.StartingPosition;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

/**
 * Basic configurations for our robot in autonomous mode. All the functionality of a teleop bot,
 * and more!
 *
 * @author Derek Helm
 */
@SuppressWarnings("unused")
public class VelRobotAuto extends VelRobot {

    /**
     * Reduction on motor gearbox.
     */
    private static final double gearbox = 40;
    /**
     * Stores the number of encoder ticks in one motor revolution.
     * For AndyMark Neverest 40's, it's 1120. The encoder tick count is actually 7 pulses per
     * revolution of the encoder disk, with 4 revolutions per cycle. Since the gearbox increases the
     * number of rotations by a factor of 40, the final count is 7 * 4 * 40 = 1120. For 20 or 60
     * reduction motors, the  number would be different.
     */
    private static final double ticksPerRevolution = 28 * gearbox;
    /**
     * Motor diameter in centimeters.
     */
    private static final double wheelDiameterCM = 10.0;

    /**
     * Motor diameter in Inches
     */
    private static final double wheelDiameterIN = 4;

    /**
     * Gear ratio between the motor and the drive wheels. Used in calculating distance.
     */
    private static final double driveGearMultiplier = 2;
//    double turnOvershootThreshold = 0.1;

    LinearOpMode mode;

    /**
     * Construct a Robot object.
     *
     * @param mode The OpMode in which the robot is being used.
     */
    public VelRobotAuto(LinearOpMode mode) {
        super(mode);
    }

//    /**
//     * Tap the beacon on the correct side.
//     * @param allianceColor The color that we are currently playing as.
//     */
//    public void tapBeacon(PublicEnums.AllianceColor allianceColor) {
//        PublicEnums.AllianceColor dominantColor;
//        double positionBeaconServo;
//
//        // Detect the color shown on the beacon's left half, and record it.
//        if (sensorColor.red() > sensorColor.blue()) {
//            dominantColor = PublicEnums.AllianceColor.RED;
//        } else {
//            dominantColor = PublicEnums.AllianceColor.BLUE;
//        }
//
//        // Tap the correct side based on the dominant color.
//        if (dominantColor == allianceColor) {
//            positionBeaconServo = VelRobotConstants.BEACON_TAP_LEFT;
//        } else {
//            positionBeaconServo = VelRobotConstants.BEACON_TAP_RIGHT;
//        }
//
//        // Trim the servo value and set the servo position.
//        positionBeaconServo = trimServoValue(positionBeaconServo);
//        servoBeaconRight.setPosition(positionBeaconServo);
//    }

    /**
     * detect white bar on ground in front of beacon
     */
    public void driveToLine(double angle, double speed, PublicEnums.GyroCorrection gyroCorrection, PublicEnums.BeaconNumber beaconNumber) {

        // Shift angle by 45 degrees, since our drive train is x-shaped and not cross-shaped
        angle += PI / 4;

        // Cut rotation in half because we don't want to spin THAT fast

        // Normalize magnitudes so that "straight forward" has a magnitude of 1
        speed *= sqrt(2);

        double sinDir = sin(angle);
        double cosDir = cos(angle);

        // None of this stuff should happen if the speed is 0.

        // Rotation is scaled down by 50% so that it doesn't completely cancel out any motors
        double multipliers[] = new double[4];
        multipliers[0] = (speed * sinDir);
        multipliers[1] = (speed * cosDir);
        multipliers[2] = (speed * -cosDir);
        multipliers[3] = (speed * -sinDir);

        double largest = abs(multipliers[0]);
        for (int i = 1; i < 4; i++) {
            if (abs(multipliers[i]) > largest)
                largest = abs(multipliers[i]);
        }

        // Only normalize multipliers if largest exceeds 1.0
        if (largest > 1.0) {
            for (int i = 0; i < 4; i++) {
                multipliers[i] = multipliers[i] / largest;
            }
        }

        // Scale if needed, 0.0 < scale < 1.0;
//        for (int i = 0; i < 4; i++) {
//            multipliers[i] = multipliers[i] * scale;
//        }
        int x = 0;
        while (isThereMat() && mode.opModeIsActive() && x == 0) {
            if (gyroCorrection == PublicEnums.GyroCorrection.YES) {
                int currentHead = startDirection;
                // TODO Fix wiring. Motors 2 and 4 are plugged into the wrong motor ports.

//if (beaconNumber == PublicEnums.BeaconNumber.ONE &&(rightBeaconUS.getUltrasonicLevel() <= 10 || leftBeaconUS.getUltrasonicLevel() <= 10)){
//        x++;
//}
                if (getGyroHeading() < currentHead) {
                    motorDrive1.setPower(multipliers[0]);
                    motorDrive3.setPower(multipliers[2]);
                    motorDrive4.setPower(multipliers[1] / 1.5);
                    motorDrive2.setPower(multipliers[3] / 1.5);
                } else if (getGyroHeading() > currentHead) {
                    motorDrive1.setPower(multipliers[0] / 1.5);
                    motorDrive3.setPower(multipliers[2] / 1.5);
                    motorDrive4.setPower(multipliers[1]);
                    motorDrive2.setPower(multipliers[3]);
                } else {
                    motorDrive1.setPower(multipliers[0]);
                    motorDrive3.setPower(multipliers[2]);
                    motorDrive4.setPower(multipliers[1]);
                    motorDrive2.setPower(multipliers[3]);
                }

            } else {

                motorDrive1.setPower(multipliers[0]);
                motorDrive3.setPower(multipliers[2]);
                motorDrive4.setPower(multipliers[1]);
                motorDrive2.setPower(multipliers[3]);
            }
        }
        if (x ==1) {
            driveToLine(VelRobotConstants.DIRECTION_WEST, 0.8, PublicEnums.GyroCorrection.NO, PublicEnums.BeaconNumber.TWO);
        }
        stopMovement();
    }
//
//    public void driveWithUS(double angle, double speed, double target) {
//
//        // Shift angle by 45 degrees, since our drive train is x-shaped and not cross-shaped
//        angle += PI / 4;
//
//        // Cut rotation in half because we don't want to spin THAT fast
//
//        // Normalize magnitudes so that "straight forward" has a magnitude of 1
//        speed *= sqrt(2);
//
//        double sinDir = sin(angle);
//        double cosDir = cos(angle);
//
//        // None of this stuff should happen if the speed is 0.
//
//        // Rotation is scaled down by 50% so that it doesn't completely cancel out any motors
//        double multipliers[] = new double[4];
//        multipliers[0] = (speed * sinDir);
//        multipliers[1] = (speed * cosDir);
//        multipliers[2] = (speed * -cosDir);
//        multipliers[3] = (speed * -sinDir);
//
//        double largest = abs(multipliers[0]);
//        for (int i = 1; i < 4; i++) {
//            if (abs(multipliers[i]) > largest)
//                largest = abs(multipliers[i]);
//        }
//
//        // Only normalize multipliers if largest exceeds 1.0
//        if (largest > 1.0) {
//            for (int i = 0; i < 4; i++) {
//                multipliers[i] = multipliers[i] / largest;
//            }
//        }
//
//        // Scale if needed, 0.0 < scale < 1.0;
////        for (int i = 0; i < 4; i++) {
////            multipliers[i] = multipliers[i] * scale;
////        }
//        int currentHead = startDirection;
//        // TODO Fix wiring. Motors 2 and 4 are plugged into the wrong motor ports.
//        double correctionScaleRight;
//        double correctionScaleLeft;
//// any errors with misalignment will get fixed when the robot squares on the wall
//        while (leftBeaconUS.getUltrasonicLevel() > target && rightBeaconUS.getUltrasonicLevel() > target && mode.opModeIsActive()) {
//            if (leftBeaconUS.getUltrasonicLevel() < rightBeaconUS.getUltrasonicLevel()) {
//                correctionScaleLeft = 1.5;
//                correctionScaleRight = 1;
//            } else if (leftBeaconUS.getUltrasonicLevel() > rightBeaconUS.getUltrasonicLevel()) {
//                correctionScaleLeft = 1.5;
//                correctionScaleRight = 1;
//            } else {
//                correctionScaleLeft = 1;
//                correctionScaleRight = 1;
//            }
//            motorDrive1.setPower(multipliers[0] / correctionScaleLeft);
//            motorDrive3.setPower(multipliers[2] / correctionScaleLeft);
//            motorDrive4.setPower(multipliers[1] / correctionScaleRight);
//            motorDrive2.setPower(multipliers[3] / correctionScaleRight);
//            mode.telemetry.addData("Robot Heading", sensorGyro.getHeading());
//            mode.telemetry.update();
//        }
//
//        stopMovement();
//    }

    public double getGroundLight() {
        return groundODS.getLightDetected();
    }

    public boolean isThereMat() {

        if (getGroundLight() - .3 <= matColorVal) {
            return true;
        } else {
            return false;
        }
    }
//    // TODO NO. WRONG.
//    private boolean detectWhiteL() {
//        return sensorColor.red() == sensorColor.blue() && sensorColor.red() == sensorColor.green();
//    }
//
//    // TODO NO. WRONG.
//    private boolean detectWhiteR() {
//        return sensorColor.red() == sensorColor.blue() && sensorColor.red() == sensorColor.green();
//    }

    /**
     * Spins all motors at the same speed. CAUTION: THIS MAKES THE ROBOT SPIN.
     *
     * @param power Speed to spin all motors.
     */
    public void setPowerAll(double power) {
        motorDrive1.setPower(power);
        motorDrive2.setPower(power);
        motorDrive3.setPower(power);
        motorDrive4.setPower(power);
    }

    public void beaconTap(PublicEnums.AllianceColor allianceColor) throws InterruptedException {
        sensorColor.enableLed(false);
        int x = 0;
        if (allianceColor == PublicEnums.AllianceColor.RED) {
            while (mode.opModeIsActive() && x < 1) {
                if (sensorColor.red() > sensorColor.blue()) {
                    servoBeaconLeft.setPosition(VelRobotConstants.BEACON_LEFT_FORWARD);
                    Thread.sleep(1000);
                    x++;

                } else if (sensorColor.blue() > sensorColor.red()) {
                    servoBeaconRight.setPosition(VelRobotConstants.BEACON_RIGHT_FORWARD);
                    Thread.sleep(1000);
                    x++;
                } else {
                    beaconServoReset();
                }
            }
        } else if (allianceColor == PublicEnums.AllianceColor.BLUE) {
            while (mode.opModeIsActive() && x < 1) {
                if (sensorColor.red() < sensorColor.blue()) {
                    servoBeaconLeft.setPosition(VelRobotConstants.BEACON_LEFT_FORWARD);
                    Thread.sleep(1000);
                    x++;

                } else if (sensorColor.blue() < sensorColor.red()) {
                    servoBeaconRight.setPosition(VelRobotConstants.BEACON_RIGHT_FORWARD);
                    Thread.sleep(1000);
                    x++;
                } else {
                    beaconServoReset();
                }
            }
        }
        beaconServoReset();
    }

    /**
     * Set the power of the left hand side drive motors.
     *
     * @param power Percentage of max power to spin.
     */
    public void setPowerLeft(double power) {
        motorDrive1.setPower(-power);
        motorDrive3.setPower(-power);
    }

    /**
     * Set the power of the right hand side drive motors.
     *
     * @param power Percentage of max speed to spin.
     */
    public void setPowerRight(double power) {
        motorDrive2.setPower(power);
        motorDrive4.setPower(power);
    }

    /**
     * Turn the robot a certain number of degrees from center.
     *
     * @param degrees Number of DEGREES to turn. Positive is counterclockwise, negative is clockwise.
     * @param speed   Speed at which to turn.
     * @throws InterruptedException Make sure that we don't get trapped in this method when interrupted.
     */
    void turnDegrees(double degrees, double speed) throws InterruptedException {

        double degreesSoFar = getGyroHeading();

        if (degrees > 180) {
//            robot.setPowerLeft(-1 * speed);
//            robot.setPowerRight(speed);
            mode.telemetry.addData("gyro1", getGyroHeading());
        } else if (degrees < 180 || degrees == 180) {
//            robot.setPowerLeft(speed);
//            robot.setPowerRight(-1 * speed);
            mode.telemetry.addData("gyro2", getGyroHeading());
        } else {
            this.setPowerAll(0);
        }
        mode.telemetry.addData("Gyro", degrees + "," + degreesSoFar);
        // For as long as the current degree measure doesn't equal the target. This will work in the clockwise and
        // counterclockwise directions, since we are comparing the absolute values
        while ((degreesSoFar) < (degrees)) {
            mode.telemetry.addData("gyrocompare", degreesSoFar = getGyroHeading());
        }

        // Stop all drive motors
//        robot.setPowerAll(0);
    }

    /**
     * Turn the robot a certain number of degrees.
     * Indicating a negative degree number will turn the robot clockwise. A positive number will
     * turn the robot counterclockwise.
     *
     * @param degrees The distance in degrees to turn.
     * @param speed   The speed at which to turn.
     */
    public void turnDegreesRight(double degrees, double speed) throws InterruptedException {

        double degreesSoFar = this.getGyroHeading();
        double degreesToGo;
        double degreesFixed;

        degreesToGo = (degreesSoFar + degrees);

        if (degreesToGo < 360) {                //right
            this.setPowerLeft(speed);
            this.setPowerRight(-1 * speed);
            while ((degreesSoFar) < (degrees)) {
                mode.telemetry.addData("gyrocompare", degreesSoFar = this.getGyroHeading());
            }
        } else if (degreesToGo > 360) {
            degreesFixed = degreesToGo - 360;
            this.setPowerLeft(speed);
            this.setPowerRight(-1 * speed);
            while ((degreesSoFar) < (degreesFixed)) {
                mode.telemetry.addData("gyrocompare", degreesSoFar = this.getGyroHeading());
            }
        } else {
            this.setPowerAll(0);
        }
    }

    public void turnDegreesLeft(double degrees, double speed) throws InterruptedException {
        double degreesSoFar = this.getGyroHeading();
        double degreesToGo;
        double degreesFixed;

        degreesToGo = (degreesSoFar - degrees);

        if (degreesToGo > 0) {                //left
            this.setPowerLeft(-1 * speed);
            this.setPowerRight(speed);
            while ((degreesSoFar) > (degrees)) {
                mode.telemetry.addData("gyrocompare", degreesSoFar = this.getGyroHeading());
            }
        } else if (degreesToGo < 0) {
            degreesFixed = 360 - degreesToGo;
            this.setPowerLeft(-1 * speed);
            this.setPowerRight(speed);
            while ((degreesSoFar) > (degreesFixed)) {
                mode.telemetry.addData("gyrocompare", degreesSoFar = this.getGyroHeading());
            }
        } else {
            this.setPowerAll(0);
        }
    }

    /**
     * Move the robot across the playing field a certain distance.
     * Indicating a negative speed or distance will cause the robot to move in reverse.
     *
     * @param distance The distance that we want to travel, in centimeters.
     * @param angle    The angle to move at, in radians.
     * @param speed    The speed at which to travel.
     */
    public void goDistanceInCm(double distance, double angle, double speed) {
        zeroEncoders();
        setMovement(angle, speed, 0.0, 1.0);
        // Track using the back left motor.
        // Why? It's the only one my fat fingers could get the plug into.
        //noinspection StatementWithEmptyBody
        while (motorDrive3.getCurrentPosition() < cmToTicks(distance)) {
        }
        stopMovement();
    }

    /**
     * Move the robot across the playing field.
     * Indicating a negative speed or distance will cause the robot to move in reverse.
     *
     * @param ticks The distance that we want to travel.
     * @param speed The speed at which to travel.
     */
    void goTicks(long ticks, double speed) throws InterruptedException {

//        long startLeft = robot.getLeftEncoder();
        long startRight = this.getDrive1Encoder();

        // Target encoder values for the left and right motors
        long targetRight = startRight + ticks;
//        long targetLeft = startLeft + ticks;

        double leftCorrect = 0.8;
        double rightCorrect = 1.0;

        if (ticks < 0) {
            // Set the drive motors to the given speed
//            robot.setPowerLeft(speed * leftCorrect);
//            robot.setPowerRight(speed * rightCorrect);
//            robot.setPowerLeft(0.85);
//            robot.setPowerRight(0.60);

            // Wait until both motors have reached the target
            while (this.getDrive1Encoder() > targetRight) {
                //TODO make telemetry work
//                mode.telemetry.addData("Data", this.getRightEncoder());
//                mode.telemetry.addData("Encoder target", targetRight);
//                mode.telemetry.addData("gyro", this.getGyroHeading());

                /* Gyro Compensation TODO Revisit this. It's wrong. */
                /*if (this.getGyroHeading() > 180) {
                    this.setPowerLeft(speed/2);
                    this.setPowerRight(1);
                } else if (this.getGyroHeading() < 180 && this.getGyroHeading() > 0) {
                    this.setPowerLeft(1);
                    this.setPowerRight(speed/2);
                } else*/
                {
                    this.setPowerLeft(speed * leftCorrect);
                    this.setPowerRight(speed * rightCorrect);
                }
            }

            // Stop the drive motors here
            this.setPowerLeft(0);
            this.setPowerRight(0);
        } else if (ticks > 0) {
            // Set the drive motors to the speed (in reverse)
            this.setPowerLeft(-speed * leftCorrect);
            this.setPowerRight(-speed * rightCorrect);
//            robot.setPowerLeft(-0.85);
//            robot.setPowerRight(-0.60);

            // Wait until both motors have reached the target
            while (this.getDrive1Encoder() < targetRight) {
//                mode.telemetry.addData("Data2", getDrive1Encoder());
//                mode.telemetry.addData("Encoder target", targetRight);
            }

            // Turn off the drive motors here
            this.setPowerLeft(0);
            this.setPowerRight(0);
        }
    }

    /**
     * Reset the encoders on all motors.
     */
    //TODO not neccasary
    public void zeroEncoders() {
        motorDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDrive3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDrive4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        try {
//            mode.idle();
//        } catch (Exception e) {
//            e.printStackTrace();
//        }

        motorDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDrive3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDrive4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    /**
     * Converts a distance in encoder ticks to a distance in inches.
     * <p>We calculate this by taking the number of ticks traveled, divided by the number of ticks
     * per revolution, and then multiplied by the gear ratio multiplier to get the number of wheel
     * rotations. Multiply one more time by the circumference of the wheels (PI*wheelDiameter).
     *
     * @param ticks long representing the distance in ticks.
     * @return that distance in inches.
     */
    public static double ticksToCm(int ticks) {
        // TODO This is wrong.
        return (ticks / ticksPerRevolution) * driveGearMultiplier * (PI * wheelDiameterCM);
    }

    /**
     * Converts a distance in inches to a distance in encoder ticks.
     * <p>We calculate this by taking the number of wheel rotations (inches/(PI*wheelDiameter))
     * multiplied by the inverse of the gear ratio, to get the number of motor rotations. Multiply
     * one more time by the number of motor encoder ticks per one motor revolution.
     *
     * @param cm double containing the distance you want to travel.
     * @return that distance in encoder ticks.
     */
    private static int cmToTicks(double cm) {
        // TODO This is wrong now.
        return (int) ((1 / driveGearMultiplier) * ticksPerRevolution * (cm / (PI * wheelDiameterCM)));
    }

    /**
     * Converts a distance in inches to a distance in encoder ticks.
     * <p>We calculate this by taking the number of wheel rotations (inches/(PI*wheelDiameter))
     * multiplied by the inverse of the gear ratio, to get the number of motor rotations. Multiply
     * one more time by the number of motor encoder ticks per one motor revolution.
     *
     * @param inches double containing the distance you want to travel.
     * @return that distance in encoder ticks.
     */
    long inchesToTicks(double inches) throws InterruptedException {
        return (long) ((1 / driveGearMultiplier) * ticksPerRevolution * (inches / (PI * wheelDiameterIN)));
    }

    /**
     * Converts a distance in encoder ticks to a distance in inches.
     * <p>We calculate this by taking the number of ticks traveled, divided by the number of ticks
     * per revolution, and then multiplied by the gear ratio multiplier to get the number of wheel
     * rotations. Multiply one more time by the circumference of the wheels (PI*wheelDiameter).
     *
     * @param ticks long representing the distance in ticks.
     * @return that distance in inches.
     */
    public double ticksToInches(long ticks) {
        return (ticks / ticksPerRevolution) * driveGearMultiplier * (PI * wheelDiameterIN);
    }

    public void driveToShootPosition(StartingPosition startingPosition) {
        if (startingPosition == StartingPosition.FAR_FROM_RAMP) {

        }
    }


//    public long getLeftEncoder() {
//        return motorDrive1.getCurrentPosition();
//    }

//    public long getRightEncoder() {
//        return motorDrive3.getCurrentPosition();
//    }

    public double getGyroHeading() {
        return sensorGyro.getHeading();
    }

    public void calibrateGyro() {
        sensorGyro.calibrate();
    }

    public boolean isGyrocalibrate() {
        return sensorGyro.isCalibrating();
    }

    public double getrawXGyro() {
        return sensorGyro.rawX();
    }

    public double getrawYGyro() {
        return sensorGyro.rawY();
    }

    public double getrawZGyro() {
        return sensorGyro.rawZ();
    }

    public OpMode getParentOpMode() {
        return mode;
    }

}
