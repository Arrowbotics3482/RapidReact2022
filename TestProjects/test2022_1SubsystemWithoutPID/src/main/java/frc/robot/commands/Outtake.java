package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class Outtake extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem intakeSubsystem;

  public Outtake(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(this.intakeSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsystem.getIntakeMotor().set(-1 * Constants.intakeMotorSpeed);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.getIntakeMotor().set(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
