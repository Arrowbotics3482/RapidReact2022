package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /** Declaration of Shooter and Transport Motors **/

  // The TalonFXs have 2048 ticks per revolution, the velocity is reported in ticks per 0.1 seconds (100 milliseconds)
  private final WPI_TalonFX shooterMotor;
  private final WPI_TalonSRX transportMotor;

  // Declaration of Encoder for SRX; Falcon Motors (FXs) have built-in encoders
  private final Encoder shooterEncoder;

  private boolean isShooting;


  public ShooterSubsystem() {
    // Initializing Shooter and Transport Motors and Inverting Shooter
    shooterMotor = new WPI_TalonFX(Constants.shooterFalconMotorID);
    shooterMotor.setInverted(true);
    transportMotor = new WPI_TalonSRX(Constants.transportSRXID);

    /*shooterMotor.configPeakOutputForward(1);
    shooterMotor.configPeakOutputReverse(1);*/
    shooterMotor.selectProfileSlot(0, 0);

    // Initializing Encoder
    shooterEncoder = new Encoder(Constants.transportSRXEncoderIDS[0], Constants.transportSRXEncoderIDS[1], false);
    shooterMotor.setNeutralMode(NeutralMode.Coast);
    isShooting = false;
  }


  public void runShooterMotor(double value)
  {
    shooterMotor.set(ControlMode.Velocity, value);
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Speed", shooterMotor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Transport Speed", transportMotor.getSelectedSensorVelocity());
    SmartDashboard.putBoolean("Shooting", isShooting);
    
  }


  @Override
  public void simulationPeriodic() {}

  
  /* Getter Methods */

  public WPI_TalonFX getShooterMotor() {
    return this.shooterMotor;
  }

  public WPI_TalonSRX getTransportMotor() {
    return this.transportMotor;
  }

  public Encoder getShooterEncoder() {
    return this.shooterEncoder;
  }

  public void setShooting(boolean isShooting)
  {
    this.isShooting = isShooting;
  }

}
