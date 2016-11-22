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

class VelRobotConstants {
    // Servo settings
    static final double BEACON_TAP_LEFT  = 0.8;
    static final double BEACON_TAP_RIGHT = 0.2;
    static final double BEACON_RESTING   = 0.5;
    public static final double TRIMM_MOTOR         = 0.88888888888;

    /** Joystick must be pushed past this poing to register as being pushed. */
    static final double MINIMUM_JOYSTICK_THRESHOLD = 0.15F;

    /** Ball grab release servo in the upright and locked position. */
    static final double SERVO_ARMS_RELEASE_RESTING = 0.0;
    /** Ball grab release servo in the open position, letting the ball grabber drop. */
    static final double SERVO_ARMS_RELEASE_OPEN = 0.0;
    /** Ball grabber in the grabbing position. */
    static final double SERVO_ARMS_GRAB_TIGHT = 0.0;
    /** Ball grabber in the loose position, i.e. not grabbing. */
    static final double SERVO_ARMS_GRAB_LOOSE = 0.0;

    /** Direction pointing forward from the robot, in radians. */
    static final double DIRECTION_NORTH = PI/2;
    /** Direction pointing backward from the robot, in radians. */
    static final double DIRECTION_SOUTH = (3*PI)/2;
    /** Direction pointing left of the robot, in radians. */
    static final double DIRECTION_EAST = 0.0;
    /** Direction pointing right of the robot, in radians. */
    static final double DIRECTION_WEST = PI;

    /** Direction pointing forward and to the right, in radians. */
    static final double DIRECTION_NORTHEAST = PI/4;
    /** Direction pointing forward and to the left, in radians. */
    static final double DIRECTION_SOUTHEAST = (7*PI)/4;
    /** Direction pointing backward and to the left, in radians. */
    static final double DIRECTION_NORTHWEST = (3*PI)/4;
    /** Direction pointing backward and to the right, in radians. */
    static final double DIRECTION_SOUTHWEST = (5*PI)/4;


}
