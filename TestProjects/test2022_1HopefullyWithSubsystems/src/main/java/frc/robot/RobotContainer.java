// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.naming.NoInitialContextException;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.commands.Intake;
import frc.robot.commands.JoyCheck;
import frc.robot.commands.Outtake;
//import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ToggleShooter;
import frc.robot.Constants.JoyType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.SerialPort;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ToggleShooter shooterCommand = new ToggleShooter();
 
  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  // controls
   // d pad top and bottom buttons for intake
  
  // drive


  // intake
  public static WPI_TalonSRX intakeMotor;

  // climb
  public static WPI_TalonFX climbMotor;

  // shooter
  public static WPI_TalonFX shooterMotor; // the talonfxs have 2048 ticks per revolution, the velocity is reported in ticks per 0.1 seconds
  public static WPI_TalonSRX[] shooterInsertMotors;
  public static MotorControllerGroup shooterInsertMotorControllerGroup;
  public static PIDController shooterPidController;

  // other
  public static AHRS navX;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // controls
    

    // drive
    

    // intake
    intakeMotor = new WPI_TalonSRX(Constants.intakeMotorID);
    
    // climb
    climbMotor = new WPI_TalonFX(Constants.climbFalconMotorID);
    
    // shooter
    shooterMotor = new WPI_TalonFX(Constants.shooterFalconMotorID);
    shooterInsertMotorControllerGroup = new MotorControllerGroup(initializeTalonArray(Constants.shooterInsertMotorIDs));
    shooterMotor.setInverted(true);

    // other
    navX = new AHRS(SerialPort.Port.kMXP); //could be: navX = new AHRS(SPI.Port.kMXP);
    navX.enableLogging(true); //For Data Output

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    for (int i = 0; i < controllers.length; i++)
    {
      topOuttake[i].whileHeld(new JoyCheck(i, JoyType.OTHER, new Outtake()));
      bottomIntake[i].whileHeld(new JoyCheck(i, JoyType.OTHER, new Intake()));
      shooterButton[i].toggleWhenPressed(new JoyCheck(i, JoyType.OTHER, new ToggleShooter()));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //final ExampleCommand sprint = new OneSecondSprint(leftDriveMotors, rightDriveMotors);
    //return sprint;
    return shooterCommand; // this was toggle shooter command
  }
  
  // method to initialize an array of WPI_TalonSRXs given an array of device ids, returns it too for initialization of a MotorControllerGroup
  public static MotorController[] initializeTalonArray(int[] deviceIDs)
  {
      MotorController[] controllers = new MotorController[deviceIDs.length];
      for (int i = 0; i < deviceIDs.length; i++)
      {
          controllers[i] = new WPI_TalonSRX(deviceIDs[i]);
      }
      return controllers;
  }
  
  
  

}
