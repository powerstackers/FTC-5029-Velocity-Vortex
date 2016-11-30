/*
 * Copyright (C) 2016 Powerstackers
 *
 * Enums used for different purposes.
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

package com.powerstackers.velocity.common.enums;

/**
 * Enumerations used to represent settings for hardware devices.
 * @author  Derek on 1/23/2016.
 */
public class PublicEnums {

    /** Represents a motor which can have one of three settings: forward, stop, and reverse.  */
    public enum MotorSetting {
        REVERSE, STOP, FORWARD
    }

    /** The color alliance that we have been placed on. */
    public enum AllianceColor {
        RED, BLUE
    }

    /** The setting of the grabber, either tight (grabbing the ball) or loose (not grabbing it). */
    public enum GrabberSetting {
        TIGHT, LOOSE
    }
}
