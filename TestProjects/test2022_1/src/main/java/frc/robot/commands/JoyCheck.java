// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.Action;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.robot.RobotContainer;

/** An example command that uses an example subsystem. */
public class JoyCheck extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private int joyID;
  private JoyType joyType;
  private CommandBase command;
  private boolean active;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public JoyCheck(int joyID, JoyType joyType, CommandBase command) {
    this.active = false;
    this.joyID = joyID;
    this.joyType = joyType;
    this.command = command;
  }

  public enum JoyType
  {
    DRIVE, OTHER
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int checkAgainst = -1;
    if (joyType == JoyType.DRIVE)
    {
      checkAgainst = RobotContainer.currentDriveControllerIndex;
    } else if (joyType == JoyType.OTHER)
    {
      checkAgainst = RobotContainer.currentOtherControllerIndex;
    }
    if(joyID == checkAgainst)
    {
      command.schedule(true);
      active = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(active)
      command.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
