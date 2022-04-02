package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.ClimbPosition;
import frc.robot.subsystems.ClimbSubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ChangeClimbPosition extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimbSubsystem climbSubsystem;

  public ChangeClimbPosition(ClimbSubsystem climbSubsystem) {
    this.climbSubsystem = climbSubsystem;
    addRequirements(this.climbSubsystem);
  }

  @Override
  public void initialize() {
    climbSubsystem.getClimbMotor().set(ControlMode.Position, climbSubsystem.getClimbMotor().getSelectedSensorPosition() - Constants.climbMotorPosition);
    climbSubsystem.setPosition(ClimbPosition.UP);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    climbSubsystem.getClimbMotor().set(ControlMode.Position, climbSubsystem.getClimbMotor().getSelectedSensorPosition() - Constants.climbMotorPosition / 2);
    climbSubsystem.setPosition(ClimbPosition.DOWN);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
