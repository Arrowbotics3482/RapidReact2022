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
  public static int currentDriveControllerIndex,
                    currentOtherControllerIndex;

  public static Joystick[] controllers;
    
  public static JoystickButton[] shooterButton,
                                 stealDriveButton,
                                 stealOtherButton,
                                 driveFBFineTuneButton,
                                 driveTurnFineTuneButton;
  public static POVButton[] topOuttake,
                            bottomIntake; // d pad top and bottom buttons for intake
  
  // drive
  public static WPI_TalonSRX[] leftDriveMotorArr, rightDriveMotorArr;
  public static MotorControllerGroup leftDriveMotors,rightDriveMotors;
  public static DifferentialDrive drive;

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

    // drive
    leftDriveMotors = new MotorControllerGroup(initializeTalonArray(Constants.leftDriveMotorIDs));
    rightDriveMotors = new MotorControllerGroup(initializeTalonArray(Constants.rightDriveMotorIDs));

    drive = new DifferentialDrive(leftDriveMotors, rightDriveMotors);
    drive.setDeadband(0);

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
  
  public static void drive()
  {
    double fb = -1 * controllers[currentDriveControllerIndex].getRawAxis(Constants.driveFBAxisID) * Constants.driveLimitCoefficient;
    double turn = controllers[currentDriveControllerIndex].getRawAxis(Constants.driveTurnAxisID) * Constants.driveLimitCoefficient;
    boolean fbFineTune = driveFBFineTuneButton[currentDriveControllerIndex].get();
    boolean turnFineTune = driveTurnFineTuneButton[currentDriveControllerIndex].get();
    if (fbFineTune)
    {
      fb *= Constants.fineFBTuneProportion;
    } else if (Math.abs(fb) <= Constants.deadbandThreshold)
    {
      fb = 0;
    }
    if (turnFineTune)
    {
      turn *= Constants.fineTurnTuneProportion;
    } else if (Math.abs(turn) <= Constants.deadbandThreshold)
    {
      turn = 0;
    }

    // System.out.println("fb: " + fbFineTune + "\nturn: " + turnFineTune + "\n____");

    // displayData();

    drive.arcadeDrive(fb, turn);
  }

}
