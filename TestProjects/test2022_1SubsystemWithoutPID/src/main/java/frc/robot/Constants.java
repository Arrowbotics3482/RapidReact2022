package frc.robot;

public final class Constants {                              
    /** Joystick IDs and Button Bindings */

    // Joystick IDs (Xbox Controllers)
    public static final int[] controllerIDs = {0, 2};

    // Button Bindings (Individual Buttons), grouped by relevance
    public static final int shooterButtonID = 1,
                            climbButtonID = 4,
                            stealDriveControlButtonID = 7,
                            stealOtherControlButtonID = 8,
                            driveFBFineTuneButtonID = 6,
                            driveTurnFineTuneButtonID = 5; // there are some issue with these ids rn but I don't have the brain to figure it out

    public static final int[] intakePOVAngles = {0, 180};

    // Forward and Turn Axis IDs
    public static final int driveFBAxisID = 4, 
                            driveTurnAxisID = 1;
    public static final double fineFBTuneProportion = 0.75,
                               fineTurnTuneProportion = 0.5;
    public enum JoyType
    {
        DRIVE, OTHER
    }


    /** Motor and Encoder IDs */

    // Drive Constants
    public static final int[] leftDriveMotorIDs = {12, 20}, //  Left Side
                              rightDriveMotorIDs = {7, 9};  //  Right Side
    public static final boolean[] leftDriveMotorInverts = {false, false},
                                  rightDriveMotorInverts = {false, false};
    public static final double driveLimitCoefficient = 0.7; // Between -1 and 1
    public static final double deadbandThreshold = 0.1;
    
    // Climb Constants
    public static final int climbFalconMotorID = 1;
    public static final double climbMotorSpeed = 1;
    public static final int climbMotorPosition = 2;
    
    public enum ClimbPosition
    {
        UP, DOWN
    }

    // Intake Constants
    public static final int intakeMotorID = 59;
    public static final double intakeMotorVoltage = 0.4; // Between -1 and 1

    // Shooter Constants
    public static final int shooterFalconMotorID = 2;
    public static final int transportSRXID = 64; // FIND THE NUMBER SOMEWHERE
    public static final int[] transportSRXEncoderIDS = {0, 1};
    
    // Values used to correct for error of Falcon motor
    public static final int shooterMaxSpeed = 22000;
    public static final double speedErrorRatio = 2.15;

    
    /*
        **************************************************************************************************88
        TESTING SHOOTER STUFF - 4/4
    */

    public static final double shooterSpeed = 0.5;

    /*
    public static final double shooterRPM = 600d,
                               shooterTP100M = shooterRPM / 600d * 2048d;
    */

    public static final double shotSpeedRunTolerance = 200; // Units are Ticks per 100 ms
    public static final double shotSpeedStopTolerance = 1.25 * shotSpeedRunTolerance;
    public static final double insertVoltage = 0.5; // Between -1 and 1

    public static final double shooterSetpoint = 1;

    // Other Constants
    public static final double wheelDistanceApart = 22.9; // Units are inches

    // PID Constants - Unused in this version of the program
    public static final double kp = 0.05,
                               kd = 0.05,
                               ki = 0;

}

