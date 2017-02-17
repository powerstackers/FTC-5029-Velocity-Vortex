package com.powerstackers.velocity.opmodes.autonomous;

import com.powerstackers.velocity.common.VelAutonomousProgram;
import com.powerstackers.velocity.common.enums.PublicEnums;
import com.powerstackers.velocity.common.enums.StartingPosition;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * @author Derek Helm
 */
@SuppressWarnings("unused")
@Autonomous(name = "Blue Auto (Backup)", group = "Powerstackers")
public class BlueAutonomous_Backup extends VelAutonomousProgram {
    public BlueAutonomous_Backup() {
        super(PublicEnums.AllianceColor.BLUE, StartingPosition.BACKUP);
    }
}
