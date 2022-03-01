// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.commands.Intake;
import frc.robot.commands.Outtake;
//import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ToggleShooter;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ToggleShooter shooterCommand = new ToggleShooter();
 
  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  

  public static WPI_TalonSRX[] leftDriveMotors;
  public static WPI_TalonSRX[] rightDriveMotors;
  

  public static MotorControllerGroup leftDriveController;
  public static MotorControllerGroup rightDriveController;

  public static DifferentialDrive drive;

  public static WPI_TalonSRX intakeMotor;

  public static WPI_TalonFX climbMotor;
  public static WPI_TalonFX shooterMotor;

  public static Joystick driveController;
  public static Joystick otherController;

  public static POVButton topOuttake; // d pad top button for ball outtake
  public static POVButton bottomIntake; // d pad buttom button for ball intake

  public static JoystickButton shooterButton;

  public static AHRS navX;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    leftDriveMotors = new WPI_TalonSRX[Constants.leftDriveMotorIDs.length];
    rightDriveMotors = new WPI_TalonSRX[Constants.rightDriveMotorIDs.length];
    
    
    for(int i = 0; i < Constants.leftDriveMotorIDs.length; i++) // left side
    {
      leftDriveMotors[i] = new WPI_TalonSRX(Constants.leftDriveMotorIDs[i]);
      leftDriveMotors[i].setInverted(Constants.leftDriveMotorInverts[i]);
    }

    for(int i = 0; i < Constants.rightDriveMotorIDs.length; i++) // right side
    {
      rightDriveMotors[i] = new WPI_TalonSRX(Constants.rightDriveMotorIDs[i]);
      rightDriveMotors[i].setInverted(Constants.rightDriveMotorInverts[i]);
    }
    

    leftDriveController = new MotorControllerGroup(leftDriveMotors);
    rightDriveController = new MotorControllerGroup(rightDriveMotors);

    drive = new DifferentialDrive(leftDriveController, rightDriveController);
    drive.setDeadband(Constants.deadbandThreshold);

    intakeMotor = new WPI_TalonSRX(Constants.intakeMotorID);
    
    driveController = new Joystick(Constants.driveControllerID);
    otherController = new Joystick(Constants.otherControllerID);

    topOuttake = new POVButton(otherController, 0); // 0 degrees
    bottomIntake = new POVButton(otherController, 180); // 180 degrees on d pad

    climbMotor = new WPI_TalonFX(Constants.climbFalconMotorID);
    shooterMotor = new WPI_TalonFX(Constants.shooterFalconMotorID);

    shooterButton = new JoystickButton(otherController, Constants.shooterButtonID); // smthn

    navX = new AHRS(SerialPort.Port.kMXP);

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
    topOuttake.whileHeld(new Outtake());
    bottomIntake.whileHeld(new Intake());
    shooterButton.toggleWhenPressed(new ToggleShooter());

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
  

}
