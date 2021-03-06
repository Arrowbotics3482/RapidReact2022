// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  //private static Timer timer = new Timer();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    //RobotContainer.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic()
  {
    /* what was happening on 2-27:
       left joystick: left/right
       right joystick: absolutely nothing
       need to get rid of arcade drive
    */
    // we dont want this RobotContainer.drive.arcadeDrive(-1 * RobotContainer.joy.getAxis(0) * Constants.driveLimitCoefficient, RobotContainer.joy.getRawAxis(5) * Constants.driveLimitCoefficient);
    //RobotContainer.intakeMotor.set(RobotContainer.joy.getRawAxis(5));
    //RobotContainer.shooterMotor.set(RobotContainer.joy.getRawAxis(1));

    /*
    Help references:
         https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj/drive/DifferentialDrive.html
         https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robotbuilder/writing-code/robotbuilder-drive-tank.html

    */
    
    /*
      FOR THE FOLLOWING LINE:
      WE NEED TO MAKE SURE THAT:
        1. THE LEFT SIDE OF arcade (1st parameter) IS THE X-AXIS ON LEFT JOYSTICK
        2. THE RIGHT SIDE OF arcade (2nd parameter) IS THE Y-AXIS ON RIGHT JOYSTICK

      need to check via DriverStation

      USE ARCADE, NOT TANK
      
    */
    
    // RobotContainer.drive.arcadeDrive();

    // DO I NEED TWO OF THESE TO DO TWO DIFFERENT JOYSTICKS
    

    //RobotContainer.intakeMotor(-1 * RobotContainer.controller_1.getRawAxis(0) * Constants.driveLimitCoefficient, RobotContainer.controller_1.getRawAxis(5) * Constants.driveLimitCoefficient);
    
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
