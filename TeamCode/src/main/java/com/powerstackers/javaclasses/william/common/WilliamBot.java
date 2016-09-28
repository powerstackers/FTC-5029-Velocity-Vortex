package com.powerstackers.javaclasses.william.common;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by root on 7/26/16.
 */
public class WilliamBot {

    private DcMotor motorFLeft;
    private DcMotor motorFRight;
    private DcMotor motorBLeft;
    private DcMotor motorBRight;

    public WilliamBot(OpMode mode) {
        motorFLeft = mode.hardwareMap.dcMotor.get("motorFLeft");
        motorFRight = mode.hardwareMap.dcMotor.get("motorFRight");
        motorBLeft = mode.hardwareMap.dcMotor.get("motorBLeft");
        motorBRight = mode.hardwareMap.dcMotor.get("motorBRight");

        motorFRight.setDirection(DcMotor.Direction.REVERSE);
        motorBRight.setDirection(DcMotor.Direction.REVERSE);


    }

    public void initializeRobot() {


    }

    public void setLeftMotorPower(double power) {
        motorFLeft.setPower(power);
        motorBLeft.setPower(power);

    }

    public  void setRightMotorPower(double power) {
        motorBRight.setPower(power);
        motorFRight.setPower(power);

    }

    public void setAllMotorsPower(double power) {
        setLeftMotorPower(power);
        setRightMotorPower(power);
    }

}
