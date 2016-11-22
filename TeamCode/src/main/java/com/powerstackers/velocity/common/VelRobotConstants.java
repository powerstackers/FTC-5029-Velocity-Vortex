/*
 * Copyright (C) 2016 Powerstackers
 *
 * Teleop code for Velocity Vortex.
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

package com.powerstackers.velocity.common;

/**
 * Created by Derek Helm
 */

public class VelRobotConstants {
    // Servo settings
    public static final double BEACON_TAP_LEFT  = 0.8;
    public static final double BEACON_TAP_RIGHT = 0.2;
    public static final double BEACON_RESTING   = 0.5;
    public static final double TRIMM_MOTOR         = 0.88888888888;

    public static final double MINIMUM_JOYSTICK_THRESHOLD = 0.15F;

    public static final double SERVO_ARMS_RELEASE_RESTING = 0.0;
    public static final double SERVO_ARMS_RELEASE_OPEN = 0.0;
    public static final double SERVO_ARMS_GRAB_TIGHT = 0.0;
    public static final double SERVO_ARMS_GRAB_LOOSE = 0.0;
}
