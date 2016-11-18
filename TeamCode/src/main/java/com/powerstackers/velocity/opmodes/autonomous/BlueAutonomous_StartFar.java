package com.powerstackers.velocity.opmodes.autonomous;

import com.powerstackers.velocity.common.VelAutonomousProgram;
import com.powerstackers.velocity.common.enums.PublicEnums;
import com.powerstackers.velocity.common.enums.StartingPosition;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * @author Jonathan
 */
@Autonomous(name = "Blue Auto Start Far", group = "Powerstackers")
@Disabled
public class BlueAutonomous_StartFar extends VelAutonomousProgram {
    public BlueAutonomous_StartFar() {
        super(PublicEnums.AllianceColor.BLUE, StartingPosition.FAR_FROM_RAMP);
    }
}
