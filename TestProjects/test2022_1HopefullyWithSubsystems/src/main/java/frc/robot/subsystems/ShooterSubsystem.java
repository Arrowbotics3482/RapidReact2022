// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final WPI_TalonFX shooterMotor; // the talonfxs have 2048 ticks per revolution, the velocity is reported in ticks per 0.1 seconds
  private final MotorControllerGroup shooterInsertMotorControllerGroup;
  private final PIDController controller; // shooter PID controller

  public ShooterSubsystem() {
    shooterMotor = new WPI_TalonFX(Constants.shooterFalconMotorID);
    shooterInsertMotorControllerGroup = new MotorControllerGroup(RobotContainer.initializeTalonArray(Constants.shooterInsertMotorIDs));
    shooterMotor.setInverted(true);
    controller = new PIDController(Constants.kp, Constants.ki, Constants.kd);
  }


  public MotorControllerGroup getShooterInsertMotorControllerGroup() {
    return this.shooterInsertMotorControllerGroup;
  }


  public PIDController getShooterPidController() {
    return this.controller;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // controller.setIntegratorRange(); // 2 double parameters. restricts how much the integral can add to the position (or velocity??)
    controller.calculate(shooterMotor.getSelectedSensorPosition(), Constants.shooterSetpoint);
  }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
