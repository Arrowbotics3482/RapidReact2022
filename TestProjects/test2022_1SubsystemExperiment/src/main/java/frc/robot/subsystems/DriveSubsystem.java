package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveSubsystem extends SubsystemBase {
  private WPI_TalonSRX[] leftDriveMotorArr, rightDriveMotorArr;
  private MotorControllerGroup leftDriveMotors,rightDriveMotors;
  private static DifferentialDrive differentialDrive;
  public DriveSubsystem() {
    leftDriveMotors = new MotorControllerGroup(RobotContainer.initializeTalonArray(Constants.leftDriveMotorIDs));
    rightDriveMotors = new MotorControllerGroup(RobotContainer.initializeTalonArray(Constants.rightDriveMotorIDs));

    differentialDrive = new DifferentialDrive(leftDriveMotors, rightDriveMotors);
    differentialDrive.setDeadband(0);
  }
  @Override
  public void periodic() {}

  public static void drive(){ 
    double fb = -1 * ControllerSubsystem.controllers[ControllerSubsystem.currentDriveControllerIndex].getRawAxis(Constants.driveFBAxisID) * Constants.driveLimitCoefficient;
    double turn = ControllerSubsystem.controllers[ControllerSubsystem.currentDriveControllerIndex].getRawAxis(Constants.driveTurnAxisID) * Constants.driveLimitCoefficient;
    boolean fbFineTune = ControllerSubsystem.driveFBFineTuneButton[ControllerSubsystem.currentDriveControllerIndex].get();
    boolean turnFineTune = ControllerSubsystem.driveTurnFineTuneButton[ControllerSubsystem.currentDriveControllerIndex].get();
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

    differentialDrive.arcadeDrive(fb, turn);
  }

  @Override
  public void simulationPeriodic() {}
  public void stuff()
  {
  }
}
