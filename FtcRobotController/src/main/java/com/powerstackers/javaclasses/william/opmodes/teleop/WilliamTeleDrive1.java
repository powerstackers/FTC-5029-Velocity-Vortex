package com.powerstackers.javaclasses.william.opmodes.teleop;

import com.powerstackers.javaclasses.william.common.WilliamBot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.swerverobotics.library.interfaces.Disabled;
import org.swerverobotics.library.interfaces.TeleOp;

/**
 * Created by root on 7/26/16.
 */
@TeleOp(name = "William TeleOp", group = "Powerstackers")
@Disabled
public class WilliamTeleDrive1 extends OpMode {

    WilliamBot robot;

    boolean aButton;


    @Override
    public void init() {
        robot = new WilliamBot(this);
    }

    @Override
    public void loop() {

        aButton = gamepad1.a;

        if (aButton) {
            robot.setAllMotorsPower(1);
        } else {
            robot.setAllMotorsPower(0);
        }

    }
}
