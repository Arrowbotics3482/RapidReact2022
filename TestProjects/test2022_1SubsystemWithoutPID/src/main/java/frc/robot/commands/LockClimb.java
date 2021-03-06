package frc.robot.commands;

import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class LockClimb extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimbSubsystem climbSubsystem;

  public LockClimb(ClimbSubsystem climbSubsystem) {
    this.climbSubsystem = climbSubsystem;
    addRequirements(climbSubsystem);
    
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    climbSubsystem.lockClimbPosition();
    climbSubsystem.getClimbMotor().setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void end(boolean interrupted) {
    climbSubsystem.unlockClimbPosition();
    climbSubsystem.getClimbMotor().setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
