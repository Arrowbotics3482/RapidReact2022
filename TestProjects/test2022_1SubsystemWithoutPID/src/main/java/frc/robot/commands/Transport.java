package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.TransportSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Transport extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final TransportSubsystem transportSubsystem;

  public Transport(TransportSubsystem transportSubsystem) {
    this.transportSubsystem = transportSubsystem;
    addRequirements(transportSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    transportSubsystem.getTransportMotor().set(Constants.transportMotorSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    transportSubsystem.getTransportMotor().set(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
