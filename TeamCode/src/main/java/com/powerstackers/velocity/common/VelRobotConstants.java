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

import static java.lang.Math.PI;

/**
 * Created by Derek Helm
 */
@SuppressWarnings("unused")
public class VelRobotConstants {
    // Servo settings
    public static final double BEACON_TAP_LEFT  = 0.8;
    public static final double BEACON_TAP_RIGHT = 0.2;
    public static final double BEACON_RESTING   = 0.5;

    /** Joystick must be pushed past this going to register as being pushed. */
    public static final double MINIMUM_JOYSTICK_THRESHOLD = 0.15f;

    /** Ball grab release servo in the upright and locked position. */
    static final double SERVO_BALL_GRAB_STOWED = 0.0;
    /** Ball grab release servo in the open position, letting the ball grabber drop. */
    static final double SERVO_BALL_GRAB_OPEN = 0.0;
    /** Ball grabber in the grabbing position. */
    static final double SERVO_BALL_GRAB_TIGHT = 0.0;

    static final double MOTOR_SHOOTER_POWER         = -0.10;
    static final double MOTOR_SHOOTER_MAX_RPM       = 1600.0;
    static final int MOTOR_SHOOTER_TARGET_RPM       = 750;
    static final int MOTOR_SHOOTER_RPM_INCREMENT    = 50;
    public static int SCHMITT_UPPER = 20;
    public static int SCHMITT_LOWER = 20;

    static final double MOTOR_LIFT_POWER    = 1.0;
    static final double MOTOR_PICKUP_POWER  = -1.0;

    static final double DIRECTION_NORTH     = PI/2;
    static final double DIRECTION_SOUTH     = (3*PI)/2;
    static final double DIRECTION_EAST      = 0.0;
    static final double DIRECTION_WEST      = PI;
    static final double DIRECTION_NORTHEAST = PI/4;
    static final double DIRECTION_SOUTHEAST = (7*PI)/4;
    static final double DIRECTION_NORTHWEST = (3*PI)/4;
    static final double DIRECTION_SOUTHWEST = (5*PI)/4;
}
