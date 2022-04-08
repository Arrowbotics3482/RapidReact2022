package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants;

public class ControllerSubsystem extends SubsystemBase {
  public static int currentDriveControllerIndex,
                    currentOtherControllerIndex;

  public static Joystick[] controllers;
    
  public static JoystickButton[] shooterButton,
                                 stealDriveButton,
                                 stealOtherButton,
                                 driveFBFineTuneButton,
                                 pidShooterButton,
                                 driveTurnFineTuneButton,
                                 shooterBackButton,
                                 climbButton;
  
  public static POVButton[] topOuttake,
                            bottomIntake,
                            transportIntakeTest,
                            transportBackTest,
                            climbUp,
                            climbDown;

  
  // creates arrays of each button and controller, initializes arrays
  public ControllerSubsystem() {

    int len = Constants.controllerIDs.length;

    controllers = new Joystick[len];
    topOuttake = new POVButton[len];
    bottomIntake = new POVButton[len];
    transportIntakeTest = new POVButton[len];
    transportBackTest = new POVButton[len];
    climbUp = new POVButton[len];
    climbDown = new POVButton[len];
    shooterBackButton = new JoystickButton[len];
    shooterButton = new JoystickButton[len];
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
      transportIntakeTest[i] = new POVButton(controllers[i], Constants.transportPOVAngles[0]);
      transportBackTest[i] = new POVButton(controllers[i], Constants.transportPOVAngles[1]);
      climbUp[i] = new POVButton(controllers[i], Constants.climbPOVAngles[0]);
      climbDown[i] = new POVButton(controllers[i], Constants.climbPOVAngles[1]);
      shooterBackButton[i] = new JoystickButton(controllers[i], Constants.shooterBackButtonID);
      shooterButton[i] = new JoystickButton(controllers[i], Constants.shooterButtonID);
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

  // method to allow for stealing controls by changing array index value when button pressed
  public static void stealControls()
  {
    for (int i = 0; i < Constants.controllerIDs.length; i++)
    {
      if (stealDriveButton[i].get())
      {
        currentDriveControllerIndex = i;
        System.out.println(i + " steals drive");
      }
      if (stealOtherButton[i].get())
      {
        currentOtherControllerIndex = i;
        System.out.println(i + " steals other");
      }
    }
  }

  @Override
  public void simulationPeriodic() {}

}
