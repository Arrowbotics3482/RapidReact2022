package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class PIDShooterSubsystem extends PIDSubsystem {
  private final WPI_TalonFX PIDshooterMotor;
  private final MotorControllerGroup PIDshooterInsertMotorControllerGroup;
  //private final Encoder PIDshooterEncoder;// = new Encoder(0, 1, false); // figure out port ID numbers

  /* private final SimpleMotorFeedforward m_shooterFeedforward =
      new SimpleMotorFeedforward(
          0.05, 12.0/5300.0); */

  public PIDShooterSubsystem() {
    super(new PIDController(Constants.kp, Constants.ki, Constants.kd));
    getController().setTolerance(50);
    PIDshooterMotor = new WPI_TalonFX(Constants.shooterFalconMotorID);
    //PIDshooterEncoder = PIDshooterMotor.getSensorCollection();
    //PIDshooterEncoder.setDistancePerPulse(1.0/1024.0);
    // setSetpoint(0);
    
    disable();
    PIDshooterMotor.setInverted(true);
    setSetpoint(Constants.shooterTP100M);
    PIDshooterMotor.setNeutralMode(NeutralMode.Brake);
    PIDshooterInsertMotorControllerGroup = new MotorControllerGroup(RobotContainer.initializeTalonArray(Constants.shooterInsertMotorIDs));
    
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    PIDshooterMotor.setVoltage(output); // + m_shooterFeedforward.calculate(setpoint)

  }

  public MotorControllerGroup getShooterInsertMotorControllerGroup() {
    return this.PIDshooterInsertMotorControllerGroup;
  }

  public WPI_TalonFX getShooterMotor() {
    return this.PIDshooterMotor;
  }

  @Override
  public void periodic() {
    super.periodic();
    // controller.setIntegratorRange(); // 2 double parameters. restricts how much the integral can add to the position (or velocity??)
  }


  @Override
  public void simulationPeriodic() {}

  public WPI_TalonFX getPIDshooterMotor() {
    return this.PIDshooterMotor;
  }


  public MotorControllerGroup getPIDshooterInsertMotorControllerGroup() {
    return this.PIDshooterInsertMotorControllerGroup;
  }

  
  /* public SimpleMotorFeedforward getM_shooterFeedforward() {
    return this.m_shooterFeedforward;
  } */

  public void setShooterNeutral()
  {    
    PIDshooterMotor.neutralOutput();
  }


  @Override
  protected double getMeasurement() {
    // TODO Auto-generated method stub
    return PIDshooterMotor.getSelectedSensorVelocity();
  }
}
