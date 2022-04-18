package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutonomousMoveBack extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private Timer timer;
  
  private double driveVolts,
                 intakePercent;
  private boolean turnDrive;

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  public AutonomousMoveBack(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    timer = new Timer();
  }

  // Set shooter motor to velocity declared in Constants
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    
    driveSubsystem.drive(0, 0);
    intakeSubsystem.getIntakeMotor().set(0);


  }

  // Checks if shooter motor speed has been significantly reduced due to contact with ball. If true, the transport motor stops.
  @Override
  public void execute() {
    driveVolts = 0;
    intakePercent = 0;
    turnDrive = false;

    if(timer.get() < Constants.timeBackwards)
    {
      driveVolts = Constants.speedBackwards * Constants.expectedMaxVolts;
      intakePercent = Constants.intakeMotorPercent / 2;
    } else if (timer.get() < Constants.timeTurning)
    {
      turnDrive = true;
      System.out.println(turnDrive);
    }
  
    driveSubsystem.driveVoltsCorrectDrift(driveVolts);
    intakeSubsystem.getIntakeMotor().set(intakePercent);
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
