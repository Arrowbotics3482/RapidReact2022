// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {                              // motors and their id numbers
    public static final int[] leftDriveMotorIDs = {20, 6}; // motor 1: 20, motor 2: 6 - left side
    public static final int[] rightDriveMotorIDs = {59, 8}; // motor 3: 59, motor 4: 8 - right side
    public static final boolean[] leftDriveMotorInverts = {false, false};
    public static final boolean[] rightDriveMotorInverts = {false, false};
    public static final int climbFalconMotorID = 1;
    public static final int shooterFalconMotorID = 2;

    public static final double driveLimitCoefficient = 1;

    public static final int[] otherMotors = {0};

    public static final int intakeMotorID = 9;
    public static final double deadbandThreshold = 0.00001;
    public static final int joystickID = 0;
    // must add another joystick id for controller 2
    public static final double wheelDistanceApart = 22.9; // inches
    public static final double talonMaxSpeed = -1; // rpm
    public static final double falconMaxSpeed = -1; // rpm

    /*
    12 - false
    20 - true

    */
}
