package frc.robot.commands;
import frc.robot.subsystems.PIDShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShooterCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Runnable m_toRun;
  private final Runnable m_toStop;
  private final PIDShooterSubsystem pidShooterSubsystem;

  public ShooterCommand(Runnable toRun, Runnable toStop, PIDShooterSubsystem requirement) {
    m_toRun = toRun;
    m_toStop = toStop;
    pidShooterSubsystem = requirement;
    addRequirements(requirement);
  }

  @Override
  public void initialize() {
    m_toRun.run();
    //System.out.println("bruh");
    //pidShooterSubsystem.getShooterMotor().set(0.3);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    m_toStop.run();
    pidShooterSubsystem.setShooterNeutral();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
