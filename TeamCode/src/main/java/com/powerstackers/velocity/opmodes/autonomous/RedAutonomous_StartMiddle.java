package com.powerstackers.velocity.opmodes.autonomous;

import com.powerstackers.velocity.common.VelAutonomousProgram;
import com.powerstackers.velocity.common.enums.PublicEnums;
import com.powerstackers.velocity.common.enums.StartingPosition;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * @author Jonathan
 */
@Autonomous(name = "Red Auto Start Middle", group = "Powerstackers")
public class RedAutonomous_StartMiddle extends VelAutonomousProgram {
    public RedAutonomous_StartMiddle() {
        super(PublicEnums.AllianceColor.RED, StartingPosition.MIDDLE);
    }
}
