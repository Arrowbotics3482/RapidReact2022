// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimbPosition;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private WPI_TalonFX climbMotor;
  private ClimbPosition position;
  
  public ClimbSubsystem() {
    climbMotor = new WPI_TalonFX(Constants.climbFalconMotorID);
    position = ClimbPosition.DOWN;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
