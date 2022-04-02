package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class Intake extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private double time;
  private final IntakeSubsystem intakeSubsystem;

  public Intake(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(this.intakeSubsystem);
    time = -1;
  }

  public Intake(IntakeSubsystem intakeSubsystem, double time)
  {
    this(intakeSubsystem);
    this.time = time;
  }

  @Override
  public void initialize() {
    intakeSubsystem.getIntakeMotor().set(Constants.intakeMotorSpeed);
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.getIntakeMotor().set(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
