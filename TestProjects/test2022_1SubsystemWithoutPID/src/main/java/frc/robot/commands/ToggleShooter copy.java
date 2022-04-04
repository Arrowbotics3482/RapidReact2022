package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class ToggleShooter extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem shooterSubsystem;
  
  public ToggleShooter(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(this.shooterSubsystem);
  }

  // Set shooter motor to velocity declared in Constants
  @Override
  public void initialize() {
    shooterSubsystem.runShooterMotor(Constants.shooterSpeed);
    //shooterSubsystem.getShooterMotor().set(0.5);
  }

  // Checks if shooter motor speed has been significantly reduced due to contact with ball. If true, the transport motor stops.
  @Override
  public void execute() {
    shooterSubsystem.runShooterMotor(Constants.shooterSpeed);
    //shooterSubsystem.getShooterMotor().set(0.5);
    if (Math.abs(shooterSubsystem.getShooterMotor().getSelectedSensorVelocity() - Constants.shooterSpeed) < Constants.shotSpeedRunTolerance) { 
      shooterSubsystem.getTransportMotor().set(Constants.insertVoltage);
    } else if (Math.abs(shooterSubsystem.getShooterMotor().getSelectedSensorVelocity() - Constants.shooterSpeed) > Constants.shotSpeedStopTolerance) {
      shooterSubsystem.getTransportMotor().set(0);      
    }
  }

  // Turns both motors off.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.getTransportMotor().set(0);
    shooterSubsystem.getShooterMotor().set(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
