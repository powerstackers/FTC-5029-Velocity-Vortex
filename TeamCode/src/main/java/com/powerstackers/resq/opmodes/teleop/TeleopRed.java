package com.powerstackers.resq.opmodes.teleop;

import com.powerstackers.resq.common.enums.PublicEnums.AllianceColor;

/**
 * @author Jonathan Thomas
 */
//@TeleOp(name = "Teleop Red", group = "Powerstackers")
public class TeleopRed extends ResqTeleop {
    public TeleopRed() {
        super(AllianceColor.RED);
    }
}