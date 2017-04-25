package com.powerstackers.velocity.opmodes.autonomous.world;

import com.powerstackers.velocity.common.VelAutonomousProgram;
import com.powerstackers.velocity.common.enums.PublicEnums;
import com.powerstackers.velocity.common.enums.StartingPosition;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by ftcpi on 4/23/2017.
 */

@Autonomous(name = "Blue Auto Start Near", group = "Powerstackers")

public class BlueAuto extends VelAutonomousProgram {

    public BlueAuto() {
        super(PublicEnums.AllianceColor.RED, StartingPosition.CLOSE_TO_RAMP);
    }

}
