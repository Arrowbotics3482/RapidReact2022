package edu.wpi.first.wpilibj.examples.hatchbottraditional.commands;

import edu.wpi.first.wpilibj.examples.hatchbottraditional.subsystems.HatchSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * runs backward for a bit
 * 
 */
public class OneSecondSprint extends CommandBase {
  // The subsystem the command runs on
  WPI_TalonSRX[] leftDriveMotors;
  WPI_TalonSRX[] rightDriveMotors;
  public OneSecondSprint(WPI_TalonSRX[] leftMotors, WPI_TalonSRX[] rightMotors){
    leftDriveMotors = leftMotors;
    rightDriveMotors = rightMotors;
  }
  @Override
  public void initialize() {
    double startTime = System.currentTimeMillis();
    for (WPI_TalonSRX motor:leftDriveMotors){
        motor.set(-0.5);
    }
    for (WPI_TalonSRX motor:rightDriveMotors){
        motor.set(-0.5);
    }
    while(System.currentTimeMillis() - startTime < 1000){
        assert true;
    }
    for (WPI_TalonSRX motor:leftDriveMotors){
        motor.set(0);
    }
    for (WPI_TalonSRX motor:rightDriveMotors){
        motor.set(0);
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}