// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants;

public class ControllerSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public static int currentDriveControllerIndex,
                    currentOtherControllerIndex;

  public static Joystick[] controllers;
    
  public static JoystickButton[] shooterButton,
                                 stealDriveButton,
                                 stealOtherButton,
                                 driveFBFineTuneButton,
                                 driveTurnFineTuneButton;
  
  public static POVButton[] topOuttake,
                            bottomIntake;
  
  
  public ControllerSubsystem() {
    int len = Constants.controllerIDs.length;

    controllers = new Joystick[len];
    topOuttake = new POVButton[len];
    bottomIntake = new POVButton[len];
    shooterButton = new JoystickButton[len];
    driveFBFineTuneButton = new JoystickButton[len];
    driveTurnFineTuneButton = new JoystickButton[len];
    stealDriveButton = new JoystickButton[len];
    stealOtherButton = new JoystickButton[len];

    for (int i = 0; i < len; i++)
    {
      controllers[i] = new Joystick(Constants.controllerIDs[i]);
      topOuttake[i] = new POVButton(controllers[i], Constants.intakePOVAngles[0]);
      bottomIntake[i] = new POVButton(controllers[i], Constants.intakePOVAngles[1]);
      shooterButton[i] = new JoystickButton(controllers[i], Constants.shooterButtonID);
      driveFBFineTuneButton[i] = new JoystickButton(controllers[i], Constants.driveFBFineTuneButtonID);
      driveTurnFineTuneButton[i] = new JoystickButton(controllers[i], Constants.driveTurnFineTuneButtonID);
      stealDriveButton[i] = new JoystickButton(controllers[i], Constants.stealDriveControlButtonID);
      stealOtherButton[i] = new JoystickButton(controllers[i], Constants.stealOtherControlButtonID);
    }

  }

  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
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
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
