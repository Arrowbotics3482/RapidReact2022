package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ShooterSubsystem extends SubsystemBase {
  private final WPI_TalonFX shooterMotor; // the talonfxs have 2048 ticks per revolution, the velocity is reported in ticks per 0.1 seconds
  private final MotorControllerGroup shooterInsertMotorControllerGroup;
  private final PIDController controller; // shooter PID controller
  private final Encoder shooterEncoder; // talon encoder

  public ShooterSubsystem() {
    shooterMotor = new WPI_TalonFX(Constants.shooterFalconMotorID);
    shooterInsertMotorControllerGroup = new MotorControllerGroup(RobotContainer.initializeTalonArray(Constants.shooterInsertMotorIDs));;
    shooterMotor.setInverted(true);
    controller = new PIDController(Constants.kp, Constants.ki, Constants.kd);
    shooterEncoder = new Encoder(0, 1, false);
  }



  public PIDController getController() {
    return this.controller;
  }


  public Encoder getShooterEncoder() {
    return this.shooterEncoder;
  }


  public MotorControllerGroup getShooterInsertMotorControllerGroup() {
    return this.shooterInsertMotorControllerGroup;
  }


  public PIDController getShooterPidController() {
    return this.controller;
  }

  public WPI_TalonFX getShooterMotor() {
    return this.shooterMotor;
  }

  public void runMotor(double value)
  {
    shooterMotor.set(ControlMode.Velocity, value);
  }

  @Override
  public void periodic() {
    // controller.setIntegratorRange(); // 2 double parameters. restricts how much the integral can add to the position (or velocity??)
    controller.calculate(shooterMotor.getSelectedSensorVelocity(), Constants.shooterSetpoint);
  }


  @Override
  public void simulationPeriodic() {}
}
