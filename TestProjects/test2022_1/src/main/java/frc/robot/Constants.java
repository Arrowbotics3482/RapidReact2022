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
    // joystick ids and button bindings
        // joystick ids
    public static final int[] controllerIDs = {0, 2};
        // button bindings
    public static final int shooterButtonID = 1,
                            climbButtonID = 4,
                            stealDriveControlButtonID = 7,
                            stealOtherControlButtonID = 8,
                            driveFBFineTuneButtonID = 6,
                            driveTurnFineTuneButtonID = 5;

    public static final int[] intakePOVAngles = {0, 180};
    public static final int driveFBAxisID = 4, 
                            driveTurnAxisID = 1; // forward and turn axis IDs
    public static final double fineTuneProportion = 3;
    

    // drive constants
    public static final int[] leftDriveMotorIDs = {12, 20}, //  - left side
                              rightDriveMotorIDs = {7, 9}; //  - right side
    public static final boolean[] leftDriveMotorInverts = {false, false},
                                  rightDriveMotorInverts = {false, false};
    public static final double driveLimitCoefficient = 0.7; // -1, 1
    public static final double deadbandThreshold = 0.1;
    
    // climb constants
    public static final int climbFalconMotorID = 1;
    
    // intake constants
    public static final int intakeMotorID = 59;
    public static final double intakeMotorSpeed = 0.5; // -1, 1

    // shooter constants
    public static final int shooterFalconMotorID = 2;
    
    public static final double shooterRPM = 1;
    public static final double shooterTP100M = shooterRPM / 600 * 2048;
    public static final double shotSpeedTolerance = 200; // in ticks per 100 ms
    public static final double insertVoltage = 0.5; // between -1 and 1
    public static final int[] shooterInsertMotorIDs = {3, 4};

    // other constants
    public static final double wheelDistanceApart = 22.9; // inches

    // PID CONSTANTS
    public static final double kp = 0,
                               kd = 0,
                               ki = 0;

}

