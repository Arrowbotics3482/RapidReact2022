// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClimbPosition;
import frc.robot.subsystems.ClimbSubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ChangeClimbPosition extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimbSubsystem climbSubsystem;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ChangeClimbPosition(ClimbSubsystem climbSubsystem) {
    this.climbSubsystem = climbSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.climbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climbSubsystem.getClimbMotor().set(ControlMode.Position, climbSubsystem.getClimbMotor().getSelectedSensorPosition() - Constants.climbMotorPosition);
    climbSubsystem.setPosition(ClimbPosition.UP);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbSubsystem.getClimbMotor().set(ControlMode.Position, climbSubsystem.getClimbMotor().getSelectedSensorPosition() - Constants.climbMotorPosition / 2);
    climbSubsystem.setPosition(ClimbPosition.DOWN);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
