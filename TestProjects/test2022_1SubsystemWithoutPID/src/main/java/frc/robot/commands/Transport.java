package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Transport extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem shooterSubsystem;

  public Transport(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(this.shooterSubsystem);
  }

  @Override
  public void initialize() {
    shooterSubsystem.getTransportMotor().set(Constants.insertVoltage);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.getTransportMotor().set(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
