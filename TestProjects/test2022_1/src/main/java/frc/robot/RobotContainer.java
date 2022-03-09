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
 
  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  // controls
  public static Joystick actualDriveController,
                         actualOtherController,
                         driveController,
                         otherController; // the two not actual controllers can be set to either of the actual controllers, effectively "stealing" the button bindings from that controller
    
  public static JoystickButton shooterButton,
                               driveStealDriveControlButton,
                               otherStealDriveControlButton,
                               driveStealOtherControlButton,
                               otherStealOtherControlButton,
                               driveFBFineTuneButton,
                               driveTurnFineTuneButton;
  public static POVButton topOuttake,
                          bottomIntake; // d pad top and bottom buttons for intake
  
  // drive
  public static WPI_TalonSRX[] leftDriveMotors, rightDriveMotors;
  public static MotorControllerGroup leftDriveController,rightDriveController;
  public static DifferentialDrive drive;

  // intake
  public static WPI_TalonSRX intakeMotor;

  // climb
  public static WPI_TalonFX climbMotor;

  // shooter
  public static WPI_TalonFX shooterMotor; // the talonfxs have 2048 ticks per revolution, the velocity is reported in ticks per 0.1 seconds
  public static WPI_TalonSRX[] shooterInsertMotors;
  public static MotorControllerGroup shooterInsertMotorControllerGroup;

  // other
  public static AHRS navX;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // controls
    actualDriveController = new Joystick(Constants.driveControllerID);
    actualOtherController = new Joystick(Constants.otherControllerID);
    driveController = actualDriveController;
    otherController = actualOtherController;
    topOuttake = new POVButton(otherController, Constants.intakePOVAngles[0]); // 0 degrees
    bottomIntake = new POVButton(otherController, Constants.intakePOVAngles[1]); // 180 degrees on d pad
    shooterButton = new JoystickButton(otherController, Constants.shooterButtonID); // smthn
    driveStealDriveControlButton = new JoystickButton(actualDriveController, Constants.stealDriveControlButtonID);
    otherStealDriveControlButton = new JoystickButton(actualOtherController, Constants.stealDriveControlButtonID);
    driveStealOtherControlButton = new JoystickButton(actualDriveController, Constants.stealOtherControlButtonID);
    otherStealOtherControlButton = new JoystickButton(actualOtherController, Constants.stealOtherControlButtonID);
    driveFBFineTuneButton = new JoystickButton(driveController, Constants.driveFBFineTuneButtonID);
    driveTurnFineTuneButton = new JoystickButton(driveController, Constants.driveTurnFineTuneButtonID);

    // drive
    leftDriveController = new MotorControllerGroup(Constants.initializeTalonArray(leftDriveMotors, Constants.leftDriveMotorIDs));
    rightDriveController = new MotorControllerGroup(Constants.initializeTalonArray(rightDriveMotors, Constants.rightDriveMotorIDs));

    drive = new DifferentialDrive(leftDriveController, rightDriveController);
    drive.setDeadband(0);

    // intake
    intakeMotor = new WPI_TalonSRX(Constants.intakeMotorID);
    
    // climb
    climbMotor = new WPI_TalonFX(Constants.climbFalconMotorID);
    
    // shooter
    shooterMotor = new WPI_TalonFX(Constants.shooterFalconMotorID);
    shooterInsertMotorControllerGroup = new MotorControllerGroup(Constants.initializeTalonArray(shooterInsertMotors, Constants.shooterInsertMotorIDs));

    // other
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
