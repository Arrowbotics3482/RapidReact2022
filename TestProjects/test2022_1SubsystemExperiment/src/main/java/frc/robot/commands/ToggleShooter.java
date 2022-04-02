package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ToggleShooter extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem shooterSubsystem;
  
  public ToggleShooter(ShooterSubsystem shooterSubsystem) {
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
