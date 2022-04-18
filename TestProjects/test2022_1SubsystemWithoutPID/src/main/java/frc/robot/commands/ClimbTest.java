package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.ClimbDirection;
import frc.robot.subsystems.ClimbSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimbTest extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimbSubsystem climbSubsystem;
  private ClimbDirection direction;

  public ClimbTest(ClimbSubsystem climbSubsystem, ClimbDirection direction) {
    this.climbSubsystem = climbSubsystem;
    this.direction = direction;
    addRequirements(this.climbSubsystem);
  }

  // Makes climb hook go upwards
  @Override
  public void initialize() {
    climbSubsystem.setTriggerControl(false);
  }

  @Override
  public void execute() {
    double set = Constants.climbMotorTestSpeed * (direction == ClimbDirection.UP ? 1 : -1);
    climbSubsystem.moveClimbNoTrigger(set);
  }

  @Override
  public void end(boolean interrupted) {
    climbSubsystem.moveClimbNoTrigger(0);
    climbSubsystem.resetInitClimbPos();
    climbSubsystem.setTriggerControl(true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
