package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShooterBack extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem shooterSubsystem;

  public ShooterBack(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    shooterSubsystem.getShooterMotor().setVoltage(-1 * Constants.desiredVoltage);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.getShooterMotor().setVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
