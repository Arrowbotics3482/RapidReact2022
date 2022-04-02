package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants;

public class ControllerSubsystem extends SubsystemBase {
  public static int currentDriveControllerIndex,
                    currentOtherControllerIndex;

  public static Joystick[] controllers;
    
  public static JoystickButton[] shooterButton,
                                 transportButton,
                                 stealDriveButton,
                                 stealOtherButton,
                                 driveFBFineTuneButton,
                                 pidShooterButton,
                                 driveTurnFineTuneButton,
                                 climbButton;
  
  public static POVButton[] topOuttake,
                            bottomIntake;
  
  
  public ControllerSubsystem() {
    int len = Constants.controllerIDs.length;

    controllers = new Joystick[len];
    topOuttake = new POVButton[len];
    bottomIntake = new POVButton[len];
    shooterButton = new JoystickButton[len];
    transportButton = new JoystickButton[len];
    driveFBFineTuneButton = new JoystickButton[len];
    driveTurnFineTuneButton = new JoystickButton[len];
    stealDriveButton = new JoystickButton[len];
    stealOtherButton = new JoystickButton[len];
    pidShooterButton = new JoystickButton[len];
    climbButton = new JoystickButton[len];

    for (int i = 0; i < len; i++)
    {
      controllers[i] = new Joystick(Constants.controllerIDs[i]);
      topOuttake[i] = new POVButton(controllers[i], Constants.intakePOVAngles[0]);
      bottomIntake[i] = new POVButton(controllers[i], Constants.intakePOVAngles[1]);
      shooterButton[i] = new JoystickButton(controllers[i], Constants.shooterButtonID);
      transportButton[i] = new JoystickButton(controllers[i], Constants.transportID);
      driveFBFineTuneButton[i] = new JoystickButton(controllers[i], Constants.driveFBFineTuneButtonID);
      driveTurnFineTuneButton[i] = new JoystickButton(controllers[i], Constants.driveTurnFineTuneButtonID);
      stealDriveButton[i] = new JoystickButton(controllers[i], Constants.stealDriveControlButtonID);
      stealOtherButton[i] = new JoystickButton(controllers[i], Constants.stealOtherControlButtonID);
      pidShooterButton[i] = new JoystickButton(controllers[i], Constants.shooterButtonID);
      climbButton[i] = new JoystickButton(controllers[i], Constants.climbButtonID);
    }

  }
  
  @Override
  public void periodic() {}

  public static void stealControls()
  {
    for (int i = 0; i < Constants.controllerIDs.length; i++)
    {
      if (stealDriveButton[i].get())
      {
        currentDriveControllerIndex = i;
        System.out.println(i + " steals alden");
      }
      if (stealOtherButton[i].get())
      {
        currentOtherControllerIndex = i;
        System.out.println(i + " steals alden");
      }
    }
  }

  @Override
  public void simulationPeriodic() {}

}
