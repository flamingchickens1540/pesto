// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team1540.delphi;

public final class Constants {
    public static final String CANIVORE_NAME = "swerve";
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.6604;
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.5842;

    public static final class Intake {
        public static final int motor = 9;
    }
    public static final class Elevator {
        public static final int leftMotor = 10;
        public static final int rightMotor = 11;
    }

}
