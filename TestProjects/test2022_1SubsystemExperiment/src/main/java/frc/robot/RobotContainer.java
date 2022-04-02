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
import frc.robot.commands.ChangeClimbPosition;
import frc.robot.commands.Intake;
import frc.robot.commands.JoyCheck;
import frc.robot.commands.Outtake;
import frc.robot.commands.PIDToggleShooter;
import frc.robot.commands.ShooterCommand;
//import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ToggleShooter;
import frc.robot.commands.Transport;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ControllerSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PIDShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;
import frc.robot.Constants.JoyType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.SerialPort;

public class RobotContainer {

  // The robot's subsystems are defined here...
  private static ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  private static ControllerSubsystem controllerSubsystem = new ControllerSubsystem();
  private static DriveSubsystem driveSubsystem = new DriveSubsystem();
  private static TransportSubsystem transportSubsystem = new TransportSubsystem();
  private static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  private static ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private static PIDShooterSubsystem pidShooterSubsystem = new PIDShooterSubsystem();

  // The robot's commands are defined here...
  private final ToggleShooter shooterCommand = new ToggleShooter(shooterSubsystem); 
  
  // Other definitions
  public static AHRS navX;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    navX = new AHRS(SerialPort.Port.kMXP); // Could be: navX = new AHRS(SPI.Port.kMXP);
    navX.enableLogging(true); //For Data Output

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    for (int i = 0; i < ControllerSubsystem.controllers.length; i++)
    {
      ControllerSubsystem.topOuttake[i].whileHeld(new JoyCheck(i, JoyType.OTHER, new Outtake(intakeSubsystem)));
      ControllerSubsystem.bottomIntake[i].whileHeld(new JoyCheck(i, JoyType.OTHER, new Intake(intakeSubsystem)));
      //ControllerSubsystem.shooterButton[i].toggleWhenPressed(new JoyCheck(i, JoyType.OTHER, new ToggleShooter(shooterSubsystem)));
      ControllerSubsystem.transportButton[i].whileHeld(new JoyCheck(i, JoyType.OTHER, new Transport(transportSubsystem)));
      ControllerSubsystem.climbButton[i].toggleWhenPressed(new JoyCheck(i, JoyType.OTHER, new ChangeClimbPosition(climbSubsystem))); 
      ControllerSubsystem.shooterButton[i].toggleWhenPressed(new JoyCheck(i, JoyType.OTHER, new ShooterCommand(pidShooterSubsystem::enable, pidShooterSubsystem::disable, pidShooterSubsystem))); //PID Command, normal Subsystem
      //ControllerSubsystem.pidShooterButton[i].toggleWhenPressed(new PIDToggleShooter(shooterSubsystem)); //PID Subsystem, normal Command
    
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
  
  // Method to initialize an array of WPI_TalonSRXs (MotorControllers) given an array of device IDs, returns it too for initialization of a MotorControllerGroup
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
