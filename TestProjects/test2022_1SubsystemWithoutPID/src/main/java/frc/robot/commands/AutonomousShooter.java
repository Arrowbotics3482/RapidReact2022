package frc.robot.commands;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ShotTarget;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutonomousShooter extends CommandBase {
  private ShotTarget target;
  private DriveSubsystem driveSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private Timer timer;
  private ToggleShooter shooterCommand;

  private double driveVolts,
                 intakeVolts;
  private boolean turnDrive;

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  public AutonomousShooter(ShooterSubsystem shooterSubsystem, ShotTarget target, DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.target = target;
    this.driveSubsystem = driveSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    shooterCommand = new ToggleShooter(shooterSubsystem, target);
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
    intakeVolts = 0;
    turnDrive = false;

    if(timer.get() < 3.5)
    {
      driveVolts = -0.25 * Constants.expectedMaxVolts;
      intakeVolts = Constants.intakeMotorVoltage / 2;
    } else if (timer.get() < 4)
    {
      turnDrive = true;
      System.out.println(turnDrive);
    } else if(timer.get() < 6 && !shooterCommand.isScheduled())
    {
      shooterCommand.schedule();
    } else if(timer.get() > 8.5 && shooterCommand.isScheduled())
    { 
      shooterCommand.cancel();
    }


    if(turnDrive)
    {
      driveSubsystem.drive(0, 0.1);
    }
    else
    {
      driveSubsystem.driveVoltsCorrectDrift(driveVolts);
    }
    intakeSubsystem.getIntakeMotor().set(intakeVolts);
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
