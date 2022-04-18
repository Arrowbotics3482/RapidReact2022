package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.highgui.HighGui;
import org.opencv.imgproc.Imgproc;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ControllerSubsystem;

public class Robot extends TimedRobot {

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private static Timer timer = new Timer();

  private Thread m_visionThread;
 
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    createCamera();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    timer.stop();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    // Runs the Autonomous Command defined in RobotContainer's
    // getAutonomousCommand() method.
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    timer.start();
  }

  @Override
  public void teleopPeriodic() {
    ControllerSubsystem.stealControls();
    RobotContainer.driveSubsystem.drive();
    RobotContainer.climbSubsystem.climb();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    // Begins timer for testing purposes.
    timer.start();
  }

  @Override
  public void testPeriodic() {}

  /** Creates Camera, called in robotInit() */
  private void createCamera() {
    m_visionThread = new Thread(
        () -> {
          // Get the UsbCamera from CameraServer
          UsbCamera camera = CameraServer.startAutomaticCapture();
          // Set the resolution
          camera.setResolution(240, 180);

          // Get a CvSink. This will capture Mats from the camera
          CvSink cvSink = CameraServer.getVideo();

          // Mats are very memory expensive. Lets reuse this Mat.
          Mat mat = new Mat();

          // Put a rectangle on the image
          Imgproc.rectangle(mat, new Point(20, 20), new Point(50, 50), new Scalar(255, 0, 0), 20);

          // Setup a CvSource. This will send images back to the Dashboard
          CvSource outputStream = CameraServer.putVideo("Rectangle", 240, 180);

          // This cannot be 'true'. The program will never exit if it is. This lets the
          // robot stop this thread when restarting robot code or deploying.
          while (!Thread.interrupted()) {
            // Tell the CvSink to grab a frame from the camera and put it in the source mat.
            // If there is an error notify the output.
            if (cvSink.grabFrame(mat) == 0) {
              // Send the output the error.
              outputStream.notifyError(cvSink.getError());
              // Skip the rest of the current iteration
              continue;
            }

            // Put a rectangle on the image
            Imgproc.line(mat, new Point(20, 20), new Point(50, 50), new Scalar(255, 0, 0), 20);

            outputStream.putFrame(mat);
            HighGui.imshow("lmao", mat);
            HighGui.waitKey();
          }
        });
    m_visionThread.setDaemon(true);
    m_visionThread.start();
  }

  /** Method that displays data collected from the Nav-X. */
  // private void displayData() {
  //   boolean zero_yaw_pressed = ControllerSubsystem.controllers[ControllerSubsystem.currentDriveControllerIndex]
  //       .getRawAxis(2) > 0.5;
  //   boolean zero_displacement_pressed = ControllerSubsystem.controllers[ControllerSubsystem.currentDriveControllerIndex]
  //       .getRawAxis(3) > 0.5;

  //   if (zero_yaw_pressed) {
  //     RobotContainer.navX.zeroYaw();
  //   }
  //   if (zero_displacement_pressed) {
  //     RobotContainer.navX.resetDisplacement();
  //   }

  //   double yaw = RobotContainer.navX.getAngle();
  //   if (yaw > maxYaw)
  //     maxYaw = yaw;

  //   /* Display 6-axis Processed Angle Data */
  //   // SmartDashboard.putNumber("IMU_TotalYaw", yaw);
  //   // SmartDashboard.putNumber("Maximum Yaw", maxYaw);
  //   // double deltaYaw = (yaw - totalYaw);
  //   // double deltaTime = (timer.get() - time);
  //   // if (Math.abs(deltaYaw/deltaTime) > 0.5) {
  //   // System.out.println("Yaw: " + 0.001 * Math.floor(1000 * deltaYaw) + "Time: " +
  //   // 0.001 * Math.floor(1000 * deltaTime));
  //   // }
  //   // SmartDashboard.putNumber("IMU_Pitch", RobotContainer.navX.getPitch());
  //   // SmartDashboard.putNumber("IMU_Roll", RobotContainer.navX.getRoll());

  //   totalYaw = yaw;
  //   time = timer.get();

  //   /* Display tilt-corrected, Magnetometer-based heading (requires */
  //   /* magnetometer calibration to be useful) */

  //   SmartDashboard.putNumber("IMU_CompassHeading", RobotContainer.navX.getCompassHeading());

  //   /* These functions are compatible w/the WPI Gyro Class, providing a simple */
  //   /* path for upgrading from the Kit-of-Parts gyro to the navx MXP */

  //   /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */

  //   SmartDashboard.putNumber("IMU_Accel_X", RobotContainer.navX.getWorldLinearAccelX());
  //   SmartDashboard.putNumber("IMU_Accel_Y", RobotContainer.navX.getWorldLinearAccelY());
  //   SmartDashboard.putBoolean("IMU_IsMoving", RobotContainer.navX.isMoving());
  //   SmartDashboard.putBoolean("IMU_IsRotating", RobotContainer.navX.isRotating());

  //   /* Display estimates of velocity/displacement. Note that these values are */
  //   /* not expected to be accurate enough for estimating robot position on a */
  //   /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
  //   /* of these errors due to single (velocity) integration and especially */
  //   /* double (displacement) integration. */

  //   // if RobotContainer.navX.getVelocityX() > get

  //   SmartDashboard.putNumber("Velocity_X", RobotContainer.navX.getVelocityX());
  //   SmartDashboard.putNumber("Velocity_Y", RobotContainer.navX.getVelocityY());
  //   SmartDashboard.putNumber("Displacement_X", RobotContainer.navX.getDisplacementX());
  //   SmartDashboard.putNumber("Displacement_Y", RobotContainer.navX.getDisplacementY());
  // }
}
