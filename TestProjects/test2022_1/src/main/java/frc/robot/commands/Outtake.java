// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


//import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/** An example command that uses an example subsystem. */
public class Outtake extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private int joyID;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Outtake(int joyID) {
    this.joyID = joyID;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (RobotContainer.isCorrectJoystick(joyID, 1))
      RobotContainer.intakeMotor.set(-1 * Constants.intakeMotorSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (RobotContainer.isCorrectJoystick(joyID, 1))
      RobotContainer.intakeMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
