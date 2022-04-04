package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.commands.ChangeClimbPosition;
import frc.robot.commands.Intake;
import frc.robot.commands.JoyCheck;
import frc.robot.commands.Outtake;
import frc.robot.commands.ToggleShooter;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ControllerSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.JoyType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.SerialPort;

public class RobotContainer {

  // The robot's subsystems are defined here.
  private static ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  private static ControllerSubsystem controllerSubsystem = new ControllerSubsystem();
  private static DriveSubsystem driveSubsystem = new DriveSubsystem();
  private static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private static ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  // The robot's commands are defined here.
  private final ToggleShooter shooterCommand = new ToggleShooter(shooterSubsystem); 
  
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
      ControllerSubsystem.topOuttake[i].whileHeld(new JoyCheck(i, JoyType.OTHER, new Outtake(intakeSubsystem)));
      ControllerSubsystem.bottomIntake[i].whileHeld(new JoyCheck(i, JoyType.OTHER, new Intake(intakeSubsystem)));
      ControllerSubsystem.shooterButton[i].toggleWhenPressed(new JoyCheck(i, JoyType.OTHER, new ToggleShooter(shooterSubsystem)));
      ControllerSubsystem.climbButton[i].toggleWhenPressed(new JoyCheck(i, JoyType.OTHER, new ChangeClimbPosition(climbSubsystem))); 
    
    }
    
  }

  /** This method supplies a Command to run during autonomousInit() */
  public Command getAutonomousCommand() {
    // Supply ToggleShooter Command to run during Autonomous
    return shooterCommand;
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
