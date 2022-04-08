package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.ClimbDirection;
import frc.robot.Constants.ClimbPosition;
import frc.robot.subsystems.ClimbSubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ChangeClimbPosition extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimbSubsystem climbSubsystem;
  private double climbInitPos;
  private boolean climbPositionDownBool = true;

  public ChangeClimbPosition(ClimbSubsystem climbSubsystem) {
    this.climbSubsystem = climbSubsystem;
    // climbInitPos = climbSubsystem.getClimbMotor().getSelectedSensorPosition();
    addRequirements(this.climbSubsystem);
  }

  // Makes climb hook go upwards
  @Override
  public void initialize() {
    // climbSubsystem.getClimbMotor().set(ControlMode.Position, (Constants.climbMotorPosition + climbSubsystem.getClimbMotor().getSelectedSensorPosition()));
    // climbPositionDownBool = climbSubsystem.getPosition() == ClimbPosition.DOWN;
    // climbSubsystem.getClimbMotor().set(climbPositionDownBool ? 1 : -1);
    // climbSubsystem.setPosition(climbPositionDownBool ? ClimbPosition.UP : ClimbPosition.DOWN);
    // if (climbSubsystem.getCounter() == 0) {
    //   climbSubsystem.getClimbMotor().setSelectedSensorPosition(0);
    // }
    // if (climbSubsystem.getCounter() < 4) {
    //   climbSubsystem.getClimbMotor().set(ControlMode.Position, Constants.climbMotorPosition * 0.52 * (climbSubsystem.getCounter() + 1) - climbSubsystem.getClimbMotor().getSelectedSensorPosition());
    // }
    // climbSubsystem.increaseCounter();
  }

  @Override
  public void execute() {
    // System.out.println(climbSubsystem.getClimbMotor().getSelectedSensorPosition() - climbInitPos);
    // climbSubsystem.getClimbMotor().set((climbPositionDownBool ? 1 : -1));
  }

  // Makes climb hook go down to half height 
  @Override
  public void end(boolean interrupted) {
  // climbSubsystem.getClimbMotor().set(0);
  // System.out.println(climbSubsystem.getClimbMotor().getSelectedSensorPosition());
  // climbSubsystem.getClimbMotor().set(ControlMode.Position, -1 * (Constants.climbMotorPosition / 2 + climbSubsystem.getClimbMotor().getSelectedSensorPosition()));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
