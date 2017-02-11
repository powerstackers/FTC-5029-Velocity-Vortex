package com.powerstackers.velocity.opmodes.autonomous;

import com.powerstackers.velocity.common.VelAutonomousProgram;
import com.powerstackers.velocity.common.enums.PublicEnums;
import com.powerstackers.velocity.common.enums.StartingPosition;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * @author Derek Helm
 */
@SuppressWarnings("unused")
@Autonomous(name = "Red Auto (Backup)", group = "Powerstackers")
public class RedAutonomous_Backup extends VelAutonomousProgram {
    public RedAutonomous_Backup() {
        super(PublicEnums.AllianceColor.RED, StartingPosition.BACKUP);
    }
}