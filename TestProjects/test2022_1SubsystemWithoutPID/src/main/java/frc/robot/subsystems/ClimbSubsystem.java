package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
  private boolean triggerControl;
  private boolean locked;

  private double maxLimit;
  
  public ClimbSubsystem(ControllerSubsystem controllerSubsystem) {
    climbMotor = new WPI_TalonFX(Constants.climbFalconMotorID);
    position = ClimbPosition.DOWN;
    climbMotor.setSelectedSensorPosition(0);
    finalPosition = 0;
    maxLimit = climbMotor.getSelectedSensorPosition() + Constants.climbSpan;
    
    climbMotor.setNeutralMode(NeutralMode.Brake);
    triggerControl = true;
    locked = false;
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
    if (triggerControl) {
      double down = ControllerSubsystem.controllers[ControllerSubsystem.currentOtherControllerIndex].getRawAxis(Constants.climbDownTriggerID);
      double up = ControllerSubsystem.controllers[ControllerSubsystem.currentOtherControllerIndex].getRawAxis(Constants.climbUpTriggerID);

      double setClimbTo = (down > up ? down : up * -1) * Constants.climbMotorSpeed;
      setClimbWithinLimits(setClimbTo);
    }
  }

  public void setClimbWithinLimits(double value)
  {
    if (checkTriggerDeadband(value) && checkSafeMove(value))
    {
      climbMotor.set(value);
    } else
    {
      climbMotor.set(0);
    }
  }

  public void moveClimbNoTrigger(double value)
  {
    if(!triggerControl)
    {
      climbMotor.set(value);
    }
  }

  public void setClimbLockPosition()
  {
    finalPosition = climbMotor.getSelectedSensorPosition();
  }

  public void lockClimbPosition() {
    if(!locked)
    {
      locked = true;
      setClimbLockPosition();
    }
    climbMotor.set(ControlMode.Position, finalPosition);
    triggerControl = false;
  }

  public void unlockClimbPosition() {
    locked = false;
    climbMotor.set(0);
    triggerControl = true;
  }

  public void resetInitClimbPos()
  {
    climbMotor.setSelectedSensorPosition(0);
    maxLimit = climbMotor.getSelectedSensorPosition() + Constants.climbSpan;
  }

  private boolean checkSafeMove(double value)
  {
    double position = climbMotor.getSelectedSensorPosition();
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

  public void setTriggerControl(boolean triggerControl)
  {
    this.triggerControl = triggerControl;
  }

  @Override
  public void simulationPeriodic() {}
}
