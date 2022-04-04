// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.AnalogGyro;

// https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervebot

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

  private final AnalogGyro analogGyro = new AnalogGyro(0);

  private final ChassisSpeeds chassisSpeed = new ChassisSpeeds(Constants.forwardSpeed, Constants.sidewaysSpeed, Constants.angularSpeed);
  
  private final Translation2d frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d backRightLocation = new Translation2d(-0.381, -0.381);

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  private final SwerveModuleState[] swerveModules = kinematics.toSwerveModuleStates(chassisSpeed);
  private final Encoder[] turningEncoders = {new Encoder(0, 1, false), new Encoder(2, 3, false), new Encoder(4, 5, false), new Encoder(6, 7, false)};
  private final SwerveModuleState frontLeftOptimized = SwerveModuleState.optimize(swerveModules[0], new Rotation2d(turningEncoders[0].getDistance()));
  private final SwerveModuleState frontRightOptimized = SwerveModuleState.optimize(swerveModules[1], new Rotation2d(turningEncoders[1].getDistance()));
  private final SwerveModuleState backLeftOptimized = SwerveModuleState.optimize(swerveModules[2], new Rotation2d(turningEncoders[2].getDistance()));
  private final SwerveModuleState backRightOptimized = SwerveModuleState.optimize(swerveModules[3], new Rotation2d(turningEncoders[3].getDistance()));

  SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, analogGyro.getRotation2d(), new Pose2d(0, 0, new Rotation2d(Math.PI)));

   /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

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
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
