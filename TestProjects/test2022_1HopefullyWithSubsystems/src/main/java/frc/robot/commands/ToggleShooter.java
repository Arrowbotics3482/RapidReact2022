// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

//import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/** An example command that uses an example subsystem. */
public class ToggleShooter extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ToggleShooter() {}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //RobotContainer.shooterMotor.set(ControlMode.Velocity, Constants.shooterTP100M);
    RobotContainer.shooterMotor.set(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(RobotContainer.shooterMotor.getSelectedSensorVelocity() - Constants.shooterTP100M) < Constants.shotSpeedTolerance)
    { 
      RobotContainer.shooterInsertMotorControllerGroup.set(Constants.insertVoltage);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooterInsertMotorControllerGroup.set(0);
    RobotContainer.shooterMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
