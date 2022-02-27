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
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.SerialPort;
//import edu.wpi.first.wpilibj.Timer;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

 
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  

  public static WPI_TalonSRX[] leftDriveMotors;
  public static WPI_TalonSRX[] rightDriveMotors;
  

  public static MotorControllerGroup leftDriveController;
  public static MotorControllerGroup rightDriveController;

  public static DifferentialDrive drive;

  public static WPI_TalonSRX intakeMotor;

  public static WPI_TalonFX climbMotor;
  public static WPI_TalonFX shooterMotor;

  public static Joystick controller_1;

  public static Joystick controller_2;

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
    
    controller_1 = new Joystick(Constants.joystickID);

    controller_2 = new Joystick(Constants.joystickID); // NEEDS NEW ID

    climbMotor = new WPI_TalonFX(Constants.climbFalconMotorID);
    shooterMotor = new WPI_TalonFX(Constants.shooterFalconMotorID);

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
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //final ExampleCommand sprint = new OneSecondSprint(leftDriveMotors, rightDriveMotors);
    //return sprint;
    return m_autoCommand;
  }
  

}
