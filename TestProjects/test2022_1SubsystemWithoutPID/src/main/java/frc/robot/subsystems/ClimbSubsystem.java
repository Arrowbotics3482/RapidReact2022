package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimbPosition;

public class ClimbSubsystem extends SubsystemBase {
  private WPI_TalonFX climbMotor;
  private ClimbPosition position;
  
  public ClimbSubsystem() {
    climbMotor = new WPI_TalonFX(Constants.climbFalconMotorID);
    position = ClimbPosition.DOWN;
  }

  // output climb position onto shuffleboard
  @Override
  public void periodic() {
    SmartDashboard.putStringArray(
      "Climb Position", 
      new String[]{this.position.toString(), String.valueOf(climbMotor.getSelectedSensorPosition())}
    );
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
