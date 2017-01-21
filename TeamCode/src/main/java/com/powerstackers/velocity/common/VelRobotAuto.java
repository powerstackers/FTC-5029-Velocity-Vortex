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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.PI;

/**
 * Basic configurations for our robot in autonomous mode. All the functionality of a teleop bot,
 * and more!
 * 
 * @author Derek Helm
 */

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
    private static final double driveGearMultiplier = 1.5;
//    double turnOvershootThreshold = 0.1;

    LinearOpMode mode;

    /**
     * Construct a Robot object.
     * @param mode The OpMode in which the robot is being used.
     */
    public VelRobotAuto(LinearOpMode mode) {
        super(mode);
    }

    /**
     * Tap the beacon on the correct side.
     * @param allianceColor The color that we are currently playing as.
     */
    public void tapBeacon(PublicEnums.AllianceColor allianceColor) {
        PublicEnums.AllianceColor dominantColor;
        double positionBeaconServo;

        // Detect the color shown on the beacon's left half, and record it.
        if (sensorColor.red() > sensorColor.blue()) {
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
        servoBeacon.setPosition(positionBeaconServo);
    }

    /**
     * detect white bar on ground in front of beacon
     */
    private boolean detectWhiteL(){
        return sensorColor.red() == sensorColor.blue() && sensorColor.red() == sensorColor.green();
    }

    private boolean detectWhiteR(){
        return sensorColor.red() == sensorColor.blue() && sensorColor.red() == sensorColor.green();
    }

    public void setPowerAll(double power) {
        motorDrive1.setPower(power);
        motorDrive2.setPower(power);
        motorDrive3.setPower(power);
        motorDrive4.setPower(power);
    }

    public void setPowerLeft(double power){
        motorDrive1.setPower(-power);
        motorDrive3.setPower(-power);
    }

    public void setPowerRight(double power){
        motorDrive2.setPower(power);
        motorDrive4.setPower(power);
    }

    /**
     * Turn the robot a certain number of degrees from center.
     * @param degrees Number of DEGREES to turn. Positive is counterclockwise, negative is clockwise.
     * @param speed Speed at which to turn.
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
//            robot.setPowerAll(0);
        }
        mode.telemetry.addData("Gyro", degrees + "," + degreesSoFar);
        // For as long as the current degree measure doesn't equal the target. This will work in the clockwise and
        // counterclockwise directions, since we are comparing the absolute values
        while ((degreesSoFar) < (degrees)) {
            mode.telemetry.addData("gyrocompare", degreesSoFar=getGyroHeading());
        }

        // Stop all drive motors
//        robot.setPowerAll(0);
    }

    /**
     * Turn the robot a certain number of degrees.
     * Indicating a negative degree number will turn the robot clockwise. A positive number will
     * turn the robot counterclockwise.
     * @param  degrees  The distance in degrees to turn.
     * @param  speed    The speed at which to turn.
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
                mode.telemetry.addData("gyrocompare", degreesSoFar=this.getGyroHeading());
            }
        } else if (degreesToGo > 360) {
            degreesFixed = degreesToGo - 360;
            this.setPowerLeft(speed);
            this.setPowerRight(-1 * speed);
            while ((degreesSoFar) < (degreesFixed)) {
                mode.telemetry.addData("gyrocompare", degreesSoFar=this.getGyroHeading());
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

        if (degreesToGo > 0 ) {                //left
            this.setPowerLeft(-1 * speed);
            this.setPowerRight(speed);
            while ((degreesSoFar) > (degrees)) {
                mode.telemetry.addData("gyrocompare", degreesSoFar=this.getGyroHeading());
            }
        } else if (degreesToGo < 0 ) {
            degreesFixed = 360 - degreesToGo;
            this.setPowerLeft(-1 * speed);
            this.setPowerRight(speed);
            while ((degreesSoFar) > (degreesFixed)) {
                mode.telemetry.addData("gyrocompare", degreesSoFar=this.getGyroHeading());
            }
        } else {
            this.setPowerAll(0);
        }
    }

    /**
     * Move the robot across the playing field a certain distance.
     * Indicating a negative speed or distance will cause the robot to move in reverse.
     * @param  distance The distance that we want to travel, in centimeters.
     * @param angle The angle to move at, in radians.
     * @param  speed The speed at which to travel.
     */
    public void goDistanceInCm(double distance, double angle, double speed) {
        zeroEncoders();
        setMovement(angle, speed, 0.0);
        // Track using the back left motor.
        // Why? It's the only one my fat fingers could get the plug into.
        //noinspection StatementWithEmptyBody
        while(motorDrive3.getCurrentPosition() < cmToTicks(distance)) {}
        stopMovement();
    }

    /**
     * Move the robot across the playing field.
     * Indicating a negative speed or distance will cause the robot to move in reverse.
     * @param  ticks The distance that we want to travel.
     * @param  speed The speed at which to travel.
     */
    public void goTicks(long ticks, double speed) {

//        long startLeft = robot.getLeftEncoder();
        long startRight = this.getDrive1Encoder();

        // Target encoder values for the left and right motors
        long targetRight = startRight + ticks;
//        long targetLeft = startLeft + ticks;

        double leftCorrect	= 1.0;
        double rightCorrect	= 0.2;

        if (ticks < 0) {
            // Set the drive motors to the given speed
//            robot.setPowerLeft(speed * leftCorrect);
//            robot.setPowerRight(speed * rightCorrect);
//            robot.setPowerLeft(0.85);
//            robot.setPowerRight(0.60);

            // Wait until both motors have reached the target
            while ( this.getDrive1Encoder() > targetRight) {
                //TODO make telemetry work
//                mode.telemetry.addData("Data", this.getRightEncoder());
//                mode.telemetry.addData("Encoder target", targetRight);
//                mode.telemetry.addData("gyro", this.getGyroHeading());

                /* Gyro Compensation */
                if (this.getGyroHeading() > 180) {
                    this.setPowerLeft(speed/2);
                    this.setPowerRight(1);
                } else if (this.getGyroHeading() < 180 && this.getGyroHeading() > 0) {
                    this.setPowerLeft(1);
                    this.setPowerRight(speed/2);
                } else {
                    this.setPowerLeft(speed * leftCorrect);
                    this.setPowerRight(speed * rightCorrect);
                }
            }

            // Stop the drive motors here
            this.setPowerLeft(0);
            this.setPowerRight(0);
        } else if (ticks > 0){
            // Set the drive motors to the speed (in reverse)
            this.setPowerLeft(-speed * leftCorrect);
            this.setPowerRight(-speed * rightCorrect);
//            robot.setPowerLeft(-0.85);
//            robot.setPowerRight(-0.60);

            // Wait until both motors have reached the target
            while( this.getDrive1Encoder() < targetRight) {
//                mode.telemetry.addData("Data2", this.getRightEncoder());
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
    private void zeroEncoders() {
        motorDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDrive3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDrive4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        try {
            mode.idle();
        } catch (Exception e) {
            e.printStackTrace();
        }

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
     * @param  ticks long representing the distance in ticks.
     * @return       that distance in inches.
     */
    public static double ticksToCm(int ticks) {
        // TODO This is wrong.
        return (ticks/ticksPerRevolution)*driveGearMultiplier*(PI*wheelDiameterCM);
    }

    /**
     * Converts a distance in inches to a distance in encoder ticks.
     * <p>We calculate this by taking the number of wheel rotations (inches/(PI*wheelDiameter))
     * multiplied by the inverse of the gear ratio, to get the number of motor rotations. Multiply
     * one more time by the number of motor encoder ticks per one motor revolution.
     * @param  cm   double containing the distance you want to travel.
     * @return      that distance in encoder ticks.
     */
    private static int cmToTicks(double cm) {
        // TODO This is wrong now.
        return (int) ((1/driveGearMultiplier)*ticksPerRevolution*(cm/(PI*wheelDiameterCM)));
    }

    /**
     * Converts a distance in inches to a distance in encoder ticks.
     * <p>We calculate this by taking the number of wheel rotations (inches/(PI*wheelDiameter))
     * multiplied by the inverse of the gear ratio, to get the number of motor rotations. Multiply
     * one more time by the number of motor encoder ticks per one motor revolution.
     * @param  inches double containing the distance you want to travel.
     * @return        that distance in encoder ticks.
     */
    public long inchesToTicks(double inches) {
        return (long) ((1/driveGearMultiplier)*ticksPerRevolution*(inches/(PI*wheelDiameterIN)));
    }

    /**
     * Converts a distance in encoder ticks to a distance in inches.
     * <p>We calculate this by taking the number of ticks traveled, divided by the number of ticks
     * per revolution, and then multiplied by the gear ratio multiplier to get the number of wheel
     * rotations. Multiply one more time by the circumference of the wheels (PI*wheelDiameter).
     * @param  ticks long representing the distance in ticks.
     * @return       that distance in inches.
     */
    public double ticksToInches(long ticks) {
        return (ticks/ticksPerRevolution)*driveGearMultiplier*(PI*wheelDiameterIN);
    }

    /**
     * Trim a servo value between the minimum and maximum ranges.
     * @param servoValue Value to trim.
     * @return A raw double with the trimmed value.
     */
    private static double trimServoValue(double servoValue) {
        return Range.clip(servoValue, 0.0, 1.0);
    }

//    public long getLeftEncoder() {
//        return motorDrive1.getCurrentPosition();
//    }

//    public long getRightEncoder() {
//        return motorDrive3.getCurrentPosition();
//    }

    public double getGyroHeading() {
        return  sensorGyro.getHeading();
    }

    public void calibrateGyro() {
        sensorGyro.calibrate();
    }

    public  boolean isGyrocalibrate() {
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
