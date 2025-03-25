// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private final CANdi myCANdi;
  private final TalonFX elevatorLeadMotor;
  private final TalonFX elevatorFollowMotor;
  private final DynamicMotionMagicVoltage mm_request =
      new DynamicMotionMagicVoltage(0, 60, 120, 1200);

  public double L1_inMotorRotations = 6.5;
  public double L2_inMotorRotations = 8.5;
  public double L3_inMotorRotations = 14.25;
  public double L4_inMotorRotations = 22.75;
  public double source_inMotorRotations = 0;

  private final double k_intakeHeightInMotorRotations = 50;

  private double currentPosSim = 0;

  /** Creates a new Elevator. */
  public Elevator() {
    myCANdi = new CANdi(17, "CANivore");

    SmartDashboard.putNumber("L1", 6.5);
    SmartDashboard.putNumber("L2", 8.5);
    SmartDashboard.putNumber("L3", 14.25);
    SmartDashboard.putNumber("L4", 22.75);
    SmartDashboard.putNumber("Source", 0);

    elevatorLeadMotor =
        new TalonFX(
            Constants.SubsystemConstants.elevator.leadMotorID,
            Constants.SubsystemConstants.elevator.kCanbus);
    elevatorFollowMotor =
        new TalonFX(
            Constants.SubsystemConstants.elevator.followMotorID,
            Constants.SubsystemConstants.elevator.kCanbus);

    // elevatorLeadMotor.getConfigurator().apply(
    //   new MotionMagicConfigs()
    //   .withMotionMagicAcceleration(1)
    //   .withMotionMagicCruiseVelocity(2));

    // Set the follow motor to follow the lead motor
    elevatorFollowMotor.setControl(new Follower(elevatorLeadMotor.getDeviceID(), false));
  }

  // Method to set the target position using Motion Magic
  public void setElevatorPosition(double targetPositionInMotorTicks) {

    if (targetPositionInMotorTicks > elevatorLeadMotor.getPosition().getValueAsDouble()) {

      mm_request.Velocity = 20;
      mm_request.Acceleration = 35;
      mm_request.Jerk = 0;
    } else {
      mm_request.Velocity = 20;
      mm_request.Acceleration = 50;
      mm_request.Jerk = 0;
    }
    if (targetPositionInMotorTicks == 0) {
      elevatorLeadMotor.setControl(
          mm_request.withPosition(targetPositionInMotorTicks).withFeedForward(-0.1));

    } else {

      elevatorLeadMotor.setControl(mm_request.withPosition(targetPositionInMotorTicks));
    }
  }

  public void setElevatorPosition(
      double targetPositionInMotorTicks, double velocity, double acceleration, double jerk) {
    currentPosSim = targetPositionInMotorTicks;
    mm_request.Velocity = velocity;
    mm_request.Acceleration = acceleration;
    mm_request.Jerk = jerk;

    elevatorLeadMotor.setControl(mm_request.withPosition(targetPositionInMotorTicks));
  }

  public void setElevatorSpeed(double speed) {

    elevatorLeadMotor.set(speed);
  }

  public boolean isAtAcceptablePosition(double targetPosition) {
    return Math.abs(elevatorLeadMotor.getPosition().getValueAsDouble() - targetPosition) < .520;
  }

  // Do we target Drive Velocity for effecting how elevator rises up

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // v_elevator.setLength(1+elevatorLeadMotor.getPosition().getValueAsDouble()*3*0.165);

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    //  v_elevator.setLength(0 + (currentPosSim / 3.0) * 0.165);
  }

  public Command setElevatorPositionCommand(double targetPositionInMotorTicks) {
    return this.runOnce(() -> setElevatorPosition(targetPositionInMotorTicks));
  }

  public Command setElevatorPositionCommand(
      double targetPositionInMotorTicks, double velocity, double acceleration, double jerk) {
    return this.runOnce(
        () -> setElevatorPosition(targetPositionInMotorTicks, velocity, acceleration, jerk));
  }

  public Command incrementElevatorPositionCommand(double incrementInMotorTicks) {
    return this.runOnce(
        () ->
            setElevatorPosition(
                elevatorLeadMotor.getPosition().getValueAsDouble() + incrementInMotorTicks));
  }

  public Command decrementElevatorPositionCommand(double decrementInMotorTicks) {
    return this.runOnce(
        () ->
            setElevatorPosition(
                elevatorLeadMotor.getPosition().getValueAsDouble() - decrementInMotorTicks));
  }

  public Command setElevatorToL1Command() {
    return this.setElevatorPositionCommand(SmartDashboard.getNumber("L1", L1_inMotorRotations));
  }

  public Command setElevatorToL2Command() {
    return this.setElevatorPositionCommand(SmartDashboard.getNumber("L2", L2_inMotorRotations));
  }

  public Command setElevatorToL3Command() {
    return this.setElevatorPositionCommand(SmartDashboard.getNumber("L3", L3_inMotorRotations));
  }

  public Command setElevatorToL4Command() {
    return this.setElevatorPositionCommand(SmartDashboard.getNumber("L4", L4_inMotorRotations));
  }

  public Command setElevatorToSourceCommand() {
    return this.setElevatorPositionCommand(SmartDashboard.getNumber("Source", 0));
  }

  // public Command setElevatorToL1Command() {
  //   return this.setElevatorPositionCommand(L1_inMotorRotations);
  // }

  // public Command setElevatorToL2Command() {
  //   return this.setElevatorPositionCommand(L2_inMotorRotations);
  // }

  // public Command setElevatorToL3Command() {
  //   return this.setElevatorPositionCommand(L3_inMotorRotations);
  // }

  // public Command setElevatorToL4Command() {
  //   return this.setElevatorPositionCommand(L4_inMotorRotations);
  // }

  // public Command setElevatorToSourceCommand() {
  //   return this.setElevatorPositionCommand(source_inMotorRotations);
  // }

  public Command setElevatorToRestCommand() {
    return this.setElevatorPositionCommand(0);
  }
}
