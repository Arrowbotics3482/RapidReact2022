package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimbDirection;
import frc.robot.Constants.ClimbPosition;

public class ClimbSubsystem extends SubsystemBase {
  private WPI_TalonFX climbMotor;
  private ClimbPosition position;
  private ControllerSubsystem controllerSubsystem;
  private double finalPosition;

  private double maxLimit;
  
  public ClimbSubsystem(ControllerSubsystem controllerSubsystem) {
    climbMotor = new WPI_TalonFX(Constants.climbFalconMotorID);
    position = ClimbPosition.DOWN;
    climbMotor.setSelectedSensorPosition(0);
    finalPosition = 0;
    maxLimit = climbMotor.getSelectedSensorPosition() + Constants.climbMotorPosition;
  }

  // output climb position onto shuffleboard
  @Override
  public void periodic() {
    // SmartDashboard.putStringArray(
    //   "Climb Position", 
    //   new String[]{this.position.toString(), String.valueOf(climbMotor.getSelectedSensorPosition())}
    // );
    SmartDashboard.putNumber("climb position", climbMotor.getSelectedSensorPosition());
    //System.out.println(climbMotor.getSelectedSensorPosition());
  }

  public void climb() {
    double down = ControllerSubsystem.controllers[ControllerSubsystem.currentOtherControllerIndex].getRawAxis(Constants.climbDownTriggerID);
    double up = ControllerSubsystem.controllers[ControllerSubsystem.currentOtherControllerIndex].getRawAxis(Constants.climbUpTriggerID);

    double setClimbTo = (down > up ? down : up * -1) * 0.5;
    setClimbWithinLimits(setClimbTo);
  }

  public void setClimbWithinLimits(double value)
  {
    if (checkTriggerDeadband(value) && checkSafeMove(value))
    {
      climbMotor.set(value);
      finalPosition = climbMotor.getSelectedSensorPosition();
      // System.out.println("2");
    } else
    {
      // climbMotor.set(ControlMode.Position, finalPosition);
      climbMotor.set(0);
    }
  }

  public void setClimbLockPosition()
  {
    finalPosition = climbMotor.getSelectedSensorPosition();
  }

  public void lockClimbPosition() {
    climbMotor.set(ControlMode.Position, finalPosition);
  }

  public void unlockClimbPosition() {
    climbMotor.set(0);
  }

  public void resetInitClimbPos()
  {
    climbMotor.setSelectedSensorPosition(0);
    maxLimit = climbMotor.getSelectedSensorPosition() + Constants.climbMotorPosition;
  }

  private boolean checkSafeMove(double value)
  {
    double position = climbMotor.getSelectedSensorPosition();
    // System.out.println("3");
    return !((position > maxLimit && value > 0) || (position < 0 && value < 0));
  }

  private boolean checkTriggerDeadband(double value)
  {
    return Math.abs(value) > Constants.triggerDeadband;
  }

  public WPI_TalonFX getClimbMotor()
  {
    return climbMotor;
  }

  public void setPosition(ClimbPosition position)
  {
    this.position = position;
  }

  public ClimbPosition getPosition()
  {
    return position;
  }

  @Override
  public void simulationPeriodic() {}
}
