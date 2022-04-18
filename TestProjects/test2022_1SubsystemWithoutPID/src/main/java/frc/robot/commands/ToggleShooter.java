package frc.robot.commands;


import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ShotTarget;
import frc.robot.subsystems.ShooterSubsystem;

public class ToggleShooter extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private ShooterSubsystem shooterSubsystem;
  private int shotSpeed;
  private double voltage;

  public ToggleShooter(ShooterSubsystem shooterSubsystem, ShotTarget target) {
    this.shooterSubsystem = shooterSubsystem;
    switch (target)
    {
      case AUTONLOWER:
        shotSpeed = Constants.autonLowerHubShotSpeed;
        break;
      case UPPER:
        shotSpeed = Constants.upperHubShotSpeed;
        break;
      case TELELOWER:
        shotSpeed = Constants.teleLowerHubShotSpeed;
        break;
    }
    addRequirements(this.shooterSubsystem);
    setVoltage(0);
  }

  // Set shooter motor to velocity declared in Constants
  @Override
  public void initialize() {
    setVoltage(2);
    // shooterSubsystem.runShooterMotor(Constants.shooterTP100M);
    //shooterSubsystem.getShooterMotor().set(0.5);
    shooterSubsystem.setShooting(true);
  }

  // Checks if shooter motor speed has been significantly reduced due to contact with ball. If true, the transport motor stops.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Shooter Speed", shooterSubsystem.getShooterMotor().getSelectedSensorVelocity());
    SmartDashboard.putNumber("Shooter Voltage", voltage);
    System.out.println(voltage);
    // shooterSubsystem.runShooterMotor(Constants.shooterTP100M);
    shooterSubsystem.getShooterMotor().setVoltage(voltage);
    //shooterSubsystem.getShooterMotor().set(ControlMode.Velocity, Constants.shooterTP100M * Constants.speedErrorRatio);
    System.out.println(shooterSubsystem.getShooterMotor().getSelectedSensorVelocity());
    if ((shooterSubsystem.getShooterMotor().getSelectedSensorVelocity() > shotSpeed - Constants.shotSpeedRunTolerance) && (voltage > 0.1)) {
      //voltage -= Constants.voltageIncrement / 200; 
      shooterSubsystem.getTransportMotor().setVoltage(Constants.insertVoltage);
    } /*else {
      voltage += Constants.voltageIncrement * ((Constants.desiredVoltage - voltage) / Constants.desiredVoltage);
    }*/

    voltage += Constants.voltageIncrement * (shotSpeed - shooterSubsystem.getShooterMotor().getSelectedSensorVelocity()) / shotSpeed;
  }

  // Turns both motors off.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.getTransportMotor().set(0);
    shooterSubsystem.getShooterMotor().set(0);
    shooterSubsystem.setShooting(false);
  }

  protected void setVoltage(double voltage) {
    this.voltage = voltage;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
