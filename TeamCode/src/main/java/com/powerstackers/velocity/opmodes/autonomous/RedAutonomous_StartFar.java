package com.powerstackers.velocity.opmodes.autonomous;

import com.powerstackers.velocity.common.VelAutonomousProgram;
import com.powerstackers.velocity.common.enums.PublicEnums;
import com.powerstackers.velocity.common.enums.StartingPosition;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * @author Jonathan
 */
@Autonomous(name = "Red Auto Start Far", group = "Powerstackers")
@Disabled
public class RedAutonomous_StartFar extends VelAutonomousProgram {
    public RedAutonomous_StartFar() {
        super(PublicEnums.AllianceColor.RED, StartingPosition.FAR_FROM_RAMP);
    }
}
