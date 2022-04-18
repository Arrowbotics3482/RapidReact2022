package frc.robot.subsystems;

import javax.lang.model.util.ElementScanner6;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveSubsystem extends SubsystemBase {
  private WPI_TalonSRX[] leftDriveMotorArr, rightDriveMotorArr;
  private MotorControllerGroup leftDriveMotors,rightDriveMotors;
  private static DifferentialDrive differentialDrive;
  private static double displacement = 0;
  // private static Timer timer = new Timer();
  // private static double lastTime = 0;

  public DriveSubsystem() {
    leftDriveMotors = new MotorControllerGroup(RobotContainer.initializeTalonArrayBrake(Constants.leftDriveMotorIDs));
    rightDriveMotors = new MotorControllerGroup(RobotContainer.initializeTalonArrayBrake(Constants.rightDriveMotorIDs));
    leftDriveMotors.setInverted(true);


    differentialDrive = new DifferentialDrive(leftDriveMotors, rightDriveMotors);
    differentialDrive.setDeadband(0);
    // timer.start();
  }
  @Override
  public void periodic() {}

  public void drive(){ 
    double fb = -1 * ControllerSubsystem.controllers[ControllerSubsystem.currentDriveControllerIndex].getRawAxis(Constants.driveFBAxisID);
    double turn = ControllerSubsystem.controllers[ControllerSubsystem.currentDriveControllerIndex].getRawAxis(Constants.driveTurnAxisID);
   
    boolean fbFineTuneBool = ControllerSubsystem.driveFBFineTuneButton[ControllerSubsystem.currentDriveControllerIndex].get();
    boolean turnFineTuneBool = ControllerSubsystem.driveTurnFineTuneButton[ControllerSubsystem.currentDriveControllerIndex].get();
    if (fbFineTuneBool)
    {
      fb *= Constants.fineFBTuneProportion;
    } else if (Math.abs(fb) <= Constants.deadbandThreshold)
    {
      fb = 0;
    } else
    {
      fb *= Constants.driveLimitCoefficient;
    }
    if (turnFineTuneBool)
    {
      turn *= Constants.fineTurnTuneProportion;
    } else if (Math.abs(turn) <= Constants.deadbandThreshold)
    {
      turn = 0;
    } else
    {
      turn *= Constants.driveLimitCoefficient;
    }

    // displacement += fb*RobotController.getBatteryVoltage()*(34.5/8.19)*(timer.get()-lastTime);
    // SmartDashboard.putNumber("Distance", displacement);

    drive(fb, turn, !turnFineTuneBool);
    // lastTime = timer.get();
  }

  public void drive(double fb, double turn, boolean correctDrift) {
    if (correctDrift)
    {
      drive(fb, correctDrift(turn, fb));
    }
    else
    {
      drive(fb, turn);
    }
  }

  public void drive(double fb, double turn) {
    differentialDrive.arcadeDrive(fb, turn);
  }

  public void driveVoltsCorrectDrift(double volts) {
    leftDriveMotors.setVoltage(volts * Constants.driveVoltageError);
    rightDriveMotors.setVoltage(volts);
  }


  public double correctDrift(double rawTurn, double rawFB)
  {
    return rawTurn + (Math.abs(rawTurn) < Constants.cancelCorrectionThreshold ? (rawFB > 0 ? Constants.forwardIncorrectionValue : Constants.backwardIncorrectionValue) : 0);
  }

  public static double getDisplacement() {
    return displacement;
  }

  @Override
  public void simulationPeriodic() {}

}
