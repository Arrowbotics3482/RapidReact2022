package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.commands.AutonomousShooter;
import frc.robot.commands.ChangeClimbPosition;
import frc.robot.commands.ClimbTest;
import frc.robot.commands.MoveIntake;
import frc.robot.commands.JoyCheck;
import frc.robot.commands.LockClimb;
import frc.robot.commands.ShooterBack;
import frc.robot.commands.ToggleShooter;
import frc.robot.commands.MoveTransport;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ControllerSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.ClimbDirection;
import frc.robot.Constants.IntakeDirection;
import frc.robot.Constants.JoyType;
import frc.robot.Constants.TransportDirection;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.SerialPort;

public class RobotContainer {

  // The robot's subsystems are defined here.
  private static ControllerSubsystem controllerSubsystem = new ControllerSubsystem();
  public static DriveSubsystem driveSubsystem = new DriveSubsystem();
  private static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private static ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public static ClimbSubsystem climbSubsystem = new ClimbSubsystem(controllerSubsystem);

  // The robot's commands are defined here.
  private final AutonomousShooter autonShooterCommand = new AutonomousShooter(shooterSubsystem, Constants.ShotTarget.UPPER, driveSubsystem, intakeSubsystem); 
  
  // Other definitions
  public static AHRS navX;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    navX = new AHRS(SerialPort.Port.kMXP); // Could be: navX = new AHRS(SPI.Port.kMXP);
    // To enable data output
    navX.enableLogging(true);

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // For each button type (Shooter button, Intake button, etc.) creates an array which carries the corresponding button on each controller
    for (int i = 0; i < ControllerSubsystem.controllers.length; i++)
    {
      ControllerSubsystem.topOuttake[i].whileHeld(new JoyCheck(i, JoyType.OTHER, new MoveIntake(intakeSubsystem, IntakeDirection.OUT)));
      ControllerSubsystem.bottomIntake[i].whileHeld(new JoyCheck(i, JoyType.OTHER, new MoveIntake(intakeSubsystem, IntakeDirection.IN)));
      ControllerSubsystem.shooterButton[i].toggleWhenPressed(new JoyCheck(i, JoyType.OTHER, new ToggleShooter(shooterSubsystem, Constants.ShotTarget.UPPER)));
      ControllerSubsystem.climbButton[i].toggleWhenPressed(new JoyCheck(i, JoyType.OTHER, new LockClimb(climbSubsystem)));
      ControllerSubsystem.transportIntakeTest[i].whileHeld(new JoyCheck(i, JoyType.OTHER, new MoveTransport(shooterSubsystem, TransportDirection.IN)));
      ControllerSubsystem.transportBackTest[i].whileHeld(new JoyCheck(i, JoyType.OTHER, new MoveTransport(shooterSubsystem, TransportDirection.OUT)));
      ControllerSubsystem.shooterBackButton[i].whileHeld(new JoyCheck(i, JoyType.OTHER, new ShooterBack(shooterSubsystem)));
      ControllerSubsystem.climbUp[i].whileHeld(new JoyCheck(i, JoyType.OTHER, new ClimbTest(climbSubsystem, ClimbDirection.UP)));
      ControllerSubsystem.climbDown[i].whileHeld(new JoyCheck(i, JoyType.OTHER, new ClimbTest(climbSubsystem, ClimbDirection.DOWN)));
    }
    
  }

  /** This method supplies a Command to run during autonomousInit() */
  public Command getAutonomousCommand() {
    // Supply ToggleShooter Command to run during Autonomous
    return autonShooterCommand;
  }

  /** Method to initialize and return an array of WPI_TalonSRXs (MotorControllers) given an array of device IDs, for initialization of a MotorControllerGroup */
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
