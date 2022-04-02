package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private WPI_TalonSRX intakeMotor;
  
  public IntakeSubsystem() {
    intakeMotor = new WPI_TalonSRX(Constants.intakeMotorID);
  }

  @Override
  public void periodic() {}

  public WPI_TalonSRX getIntakeMotor()
  {
    return intakeMotor;
  }

  @Override
  public void simulationPeriodic() {}
}
