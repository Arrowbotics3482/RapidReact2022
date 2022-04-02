package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.JoyType;
import frc.robot.subsystems.ControllerSubsystem;

public class JoyCheck extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private int joyID;
  private JoyType joyType;
  private Command command;
  public JoyCheck(int joyID, JoyType joyType, Command command) {
    this.joyID = joyID;
    this.joyType = joyType;
    this.command = command;
  }

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

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    if(command.isScheduled())
      command.cancel();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
