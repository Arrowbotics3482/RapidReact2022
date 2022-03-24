// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.JoyType;
import frc.robot.subsystems.ControllerSubsystem;
import frc.robot.subsystems.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class JoyCheck extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private int joyID;
  private JoyType joyType;
  private Command command;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public JoyCheck(int joyID, JoyType joyType, Command command) {
    this.joyID = joyID;
    this.joyType = joyType;
    this.command = command;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int checkAgainst = -1;

    switch (joyType)
    {
      case DRIVE:
        checkAgainst = ControllerSubsystem.currentDriveControllerIndex;
        break;
      case OTHER:
        checkAgainst = ControllerSubsystem.currentOtherControllerIndex;
    }
    
    if(joyID == checkAgainst)
      command.schedule(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(command.isScheduled())
      command.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
