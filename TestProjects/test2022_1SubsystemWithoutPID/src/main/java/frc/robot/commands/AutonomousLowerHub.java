package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ShotTarget;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutonomousLowerHub extends CommandBase {
  private ShotTarget target;
  private DriveSubsystem driveSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private Timer timer;
  private ToggleShooter shooterCommand;

  private double driveVolts,
                 intakePercent;

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  public AutonomousLowerHub(ShooterSubsystem shooterSubsystem, ShotTarget target, DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.target = target;
    this.driveSubsystem = driveSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    shooterCommand = new ToggleShooter(shooterSubsystem, target);
    timer = new Timer();
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    
    driveSubsystem.drive(0, 0);
    intakeSubsystem.getIntakeMotor().set(0);

    shooterCommand.schedule();
  }

  @Override
  public void execute() {
    driveVolts = 0;
    intakePercent = 0;

    if(timer.get() < 3) {
      intakePercent = Constants.intakeMotorPercent;     
    } 
    else if(timer.get() < 6.5) {
      if (shooterCommand.isScheduled())
        shooterCommand.cancel();
      driveVolts = Constants.speedBackwards * Constants.expectedMaxVolts;
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
