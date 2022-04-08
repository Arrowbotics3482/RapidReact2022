package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.TransportDirection;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveTransport extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem shooterSubsystem;
  private final TransportDirection direction;

  public MoveTransport(ShooterSubsystem shooterSubsystem, TransportDirection direction) {
    this.shooterSubsystem = shooterSubsystem;
    this.direction = direction;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    shooterSubsystem.getTransportMotor().setVoltage(Constants.insertVoltage * (direction == TransportDirection.IN ? 1 : -1));
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
