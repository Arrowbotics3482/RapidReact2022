// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public static WPI_TalonSRX[] leftDriveMotorArr, rightDriveMotorArr;
  public static MotorControllerGroup leftDriveMotors,rightDriveMotors;
  public static DifferentialDrive drive;
  public DriveSubsystem() {
    leftDriveMotors = new MotorControllerGroup(RobotContainer.initializeTalonArray(Constants.leftDriveMotorIDs));
    rightDriveMotors = new MotorControllerGroup(RobotContainer.initializeTalonArray(Constants.rightDriveMotorIDs));

    drive = new DifferentialDrive(leftDriveMotors, rightDriveMotors);
    drive.setDeadband(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static void drive(){ 
    double fb = -1 * ControllerSubsystem.controllers[ControllerSubsystem.currentDriveControllerIndex].getRawAxis(Constants.driveFBAxisID) * Constants.driveLimitCoefficient;
    double turn = ControllerSubsystem.controllers[ControllerSubsystem.currentDriveControllerIndex].getRawAxis(Constants.driveTurnAxisID) * Constants.driveLimitCoefficient;
    boolean fbFineTune = ControllerSubsystem.driveFBFineTuneButton[ControllerSubsystem.currentDriveControllerIndex].get();
    boolean turnFineTune = ControllerSubsystem.driveTurnFineTuneButton[ControllerSubsystem.currentDriveControllerIndex].get();
    if (fbFineTune)
    {
      fb *= Constants.fineFBTuneProportion;
    } else if (Math.abs(fb) <= Constants.deadbandThreshold)
    {
      fb = 0;
    }
    if (turnFineTune)
    {
      turn *= Constants.fineTurnTuneProportion;
    } else if (Math.abs(turn) <= Constants.deadbandThreshold)
    {
      turn = 0;
    }

    // System.out.println("fb: " + fbFineTune + "\nturn: " + turnFineTune + "\n____");

    // displayData();

    drive.arcadeDrive(fb, turn);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
