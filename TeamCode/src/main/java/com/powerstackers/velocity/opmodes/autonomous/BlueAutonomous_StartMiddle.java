/*
 * Copyright (C) 2016 Powerstackers
 *
 * Code to run our 2016-17 robot.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

package com.powerstackers.velocity.opmodes.autonomous;

import com.powerstackers.velocity.common.VelAutonomousProgram;
import com.powerstackers.velocity.common.enums.PublicEnums;
import com.powerstackers.velocity.common.enums.StartingPosition;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * @author Jonathan Thomas
 *
 */
@Autonomous(name = "Red Auto Start Middle", group = "Powerstackers")
public class BlueAutonomous_StartMiddle extends VelAutonomousProgram {
    public BlueAutonomous_StartMiddle() {
        super(PublicEnums.AllianceColor.BLUE, StartingPosition.MIDDLE);
    }
}