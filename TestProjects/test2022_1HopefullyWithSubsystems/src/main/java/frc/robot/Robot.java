package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ControllerSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private static Timer timer = new Timer();

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }  
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic()
  {
    ControllerSubsystem.stealControls();
    DriveSubsystem.drive();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    timer.start();
    timer.stop();
    displayData();
  }

  @Override
  public void testPeriodic() {}

  private void displayData() {
    boolean zero_yaw_pressed = ControllerSubsystem.controllers[ControllerSubsystem.currentDriveControllerIndex].getTrigger();
    if (zero_yaw_pressed) {
      RobotContainer.navX.zeroYaw();
    }

    /* Display 6-axis Processed Angle Data */
    SmartDashboard.putBoolean("IMU_Connected", RobotContainer.navX.isConnected());
    SmartDashboard.putBoolean("IMU_IsCalibrating", RobotContainer.navX.isCalibrating());
    SmartDashboard.putNumber("IMU_Yaw", RobotContainer.navX.getYaw());
    SmartDashboard.putNumber("IMU_Pitch", RobotContainer.navX.getPitch());
    SmartDashboard.putNumber("IMU_Roll", RobotContainer.navX.getRoll());

    /* Display tilt-corrected, Magnetometer-based heading (requires */
    /* magnetometer calibration to be useful) */

    SmartDashboard.putNumber("IMU_CompassHeading", RobotContainer.navX.getCompassHeading());

    /* Display 9-axis Heading (requires magnetometer calibration to be useful) */
    SmartDashboard.putNumber("IMU_FusedHeading", RobotContainer.navX.getFusedHeading());

    /* These functions are compatible w/the WPI Gyro Class, providing a simple */
    /* path for upgrading from the Kit-of-Parts gyro to the navx MXP */

    SmartDashboard.putNumber("IMU_TotalYaw", RobotContainer.navX.getAngle());
    SmartDashboard.putNumber("IMU_YawRateDPS", RobotContainer.navX.getRate());

    /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */

    SmartDashboard.putNumber("IMU_Accel_X", RobotContainer.navX.getWorldLinearAccelX());
    SmartDashboard.putNumber("IMU_Accel_Y", RobotContainer.navX.getWorldLinearAccelY());
    SmartDashboard.putBoolean("IMU_IsMoving", RobotContainer.navX.isMoving());
    SmartDashboard.putBoolean("IMU_IsRotating", RobotContainer.navX.isRotating());

    /* Display estimates of velocity/displacement. Note that these values are */
    /* not expected to be accurate enough for estimating robot position on a */
    /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
    /* of these errors due to single (velocity) integration and especially */
    /* double (displacement) integration. */

    SmartDashboard.putNumber("Velocity_X", RobotContainer.navX.getVelocityX());
    SmartDashboard.putNumber("Velocity_Y", RobotContainer.navX.getVelocityY());
    SmartDashboard.putNumber("Displacement_X", RobotContainer.navX.getDisplacementX());
    SmartDashboard.putNumber("Displacement_Y", RobotContainer.navX.getDisplacementY());

    /* Display Raw Gyro/Accelerometer/Magnetometer Values */
    /* NOTE: These values are not normally necessary, but are made available */
    /* for advanced users. Before using this data, please consider whether */
    /* the processed data (see above) will suit your needs. */

    SmartDashboard.putNumber("RawGyro_X", RobotContainer.navX.getRawGyroX());
    SmartDashboard.putNumber("RawGyro_Y", RobotContainer.navX.getRawGyroY());
    SmartDashboard.putNumber("RawGyro_Z", RobotContainer.navX.getRawGyroZ());
    SmartDashboard.putNumber("RawAccel_X", RobotContainer.navX.getRawAccelX());
    SmartDashboard.putNumber("RawAccel_Y", RobotContainer.navX.getRawAccelY());
    SmartDashboard.putNumber("RawAccel_Z", RobotContainer.navX.getRawAccelZ());
    SmartDashboard.putNumber("RawMag_X", RobotContainer.navX.getRawMagX());
    SmartDashboard.putNumber("RawMag_Y", RobotContainer.navX.getRawMagY());
    SmartDashboard.putNumber("RawMag_Z", RobotContainer.navX.getRawMagZ());
    SmartDashboard.putNumber("IMU_Temp_C", RobotContainer.navX.getTempC());
    SmartDashboard.putNumber("IMU_Timestamp", RobotContainer.navX.getLastSensorTimestamp());

    /* Omnimount Yaw Axis Information */
    /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount */
    AHRS.BoardYawAxis yaw_axis = RobotContainer.navX.getBoardYawAxis();
    SmartDashboard.putString("YawAxisDirection", yaw_axis.up ? "Up" : "Down");
    SmartDashboard.putNumber("YawAxis", yaw_axis.board_axis.getValue());

    /* Sensor Board Information */
    SmartDashboard.putString("FirmwareVersion", RobotContainer.navX.getFirmwareVersion());

    /* Quaternion Data */
    /* Quaternions are fascinating, and are the most compact representation of */
    /* orientation data. All of the Yaw, Pitch and Roll Values can be derived */
    /* from the Quaternions. If interested in motion processing, knowledge of */
    /* Quaternions is highly recommended. */
    SmartDashboard.putNumber("QuaternionW", RobotContainer.navX.getQuaternionW());
    SmartDashboard.putNumber("QuaternionX", RobotContainer.navX.getQuaternionX());
    SmartDashboard.putNumber("QuaternionY", RobotContainer.navX.getQuaternionY());
    SmartDashboard.putNumber("QuaternionZ", RobotContainer.navX.getQuaternionZ());

    /* Connectivity Debugging Support */
    SmartDashboard.putNumber("IMU_Byte_Count", RobotContainer.navX.getByteCount());
    SmartDashboard.putNumber("IMU_Update_Count", RobotContainer.navX.getUpdateCount());
  }
}
