package com.powerstackers.velocity.opmodes.autonomous.world;

import com.powerstackers.velocity.common.VelAutonomousProgram;
import com.powerstackers.velocity.common.VelAutonomousProgramWorld;
import com.powerstackers.velocity.common.enums.PublicEnums;
import com.powerstackers.velocity.common.enums.StartingPosition;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by ftcpi on 4/23/2017.
 */
@Autonomous(name = "Red Auto", group = "Powerstackers")

public class RedAuto extends VelAutonomousProgramWorld {

        public RedAuto() {
            super(PublicEnums.AllianceColor.RED, StartingPosition.CLOSE_TO_RAMP);
        }

}
