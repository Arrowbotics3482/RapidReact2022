package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeDirection;
import frc.robot.subsystems.IntakeSubsystem;

public class MoveIntake extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private double time;
  private final IntakeSubsystem intakeSubsystem;
  private final IntakeDirection direction;
  public MoveIntake(IntakeSubsystem intakeSubsystem, IntakeDirection direction) {
    this.intakeSubsystem = intakeSubsystem;
    this.direction = direction;
    addRequirements(this.intakeSubsystem);
    time = -1;
  }

  public MoveIntake(IntakeSubsystem intakeSubsystem, double time, IntakeDirection direction)
  {
    this(intakeSubsystem, direction);
    this.time = time;
  }

  // Sets intake to voltage percentage defined in Constants
  @Override
  public void initialize() {
    intakeSubsystem.getIntakeMotor().set(Constants.intakeMotorVoltage * (direction == IntakeDirection.IN ? 1 : -1));
  }

  @Override
  public void execute() {

  }

  // Turns intake motor off
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.getIntakeMotor().set(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
