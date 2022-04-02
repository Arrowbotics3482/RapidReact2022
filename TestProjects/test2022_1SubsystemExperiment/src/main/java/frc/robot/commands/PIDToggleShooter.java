package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.PIDShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class PIDToggleShooter extends PIDCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem shooterSubsystem;
  
  public PIDToggleShooter(ShooterSubsystem shooterSubsystem) {
    super(
      new PIDController(Constants.kp, Constants.ki, Constants.kd),
      shooterSubsystem.getShooterEncoder() :: getDistance,
      Constants.shooterSetpoint,
      output -> shooterSubsystem.runMotor(output),
      shooterSubsystem
    );
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(this.shooterSubsystem);
  }

  @Override
  public void initialize() {
    //RobotContainer.shooterMotor.set(ControlMode.Velocity, Constants.shooterTP100M);
    shooterSubsystem.getShooterMotor().set(1);
  }

  @Override
  public void execute() {
    if (Math.abs(shooterSubsystem.getShooterMotor().getSelectedSensorVelocity() - Constants.shooterTP100M) < Constants.shotSpeedTolerance)
    { 
      shooterSubsystem.getShooterInsertMotorControllerGroup().set(Constants.insertVoltage);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.getShooterInsertMotorControllerGroup().set(0);
    shooterSubsystem.getShooterMotor().set(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
