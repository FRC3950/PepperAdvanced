// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private final CANdi myCANdi;
  public final TalonFX elevatorLeadMotor;
  private final TalonFX elevatorFollowMotor;
  private final DynamicMotionMagicVoltage mm_request =
      new DynamicMotionMagicVoltage(0, 200, 135, 0);

  public double L1_inMotorRotations = 6.5;
  public double L2_inMotorRotations = 8.55;
  public double L3_inMotorRotations = 14.25;
  public double L4_inMotorRotations = 23.4;
  public BooleanSupplier LimitPressed;

  private double currentPosSim = 0;

  /** Creates a new Elevator. */
  public Elevator() {
    myCANdi = new CANdi(17, "CANivore");
    LimitPressed = () -> myCANdi.getS1Closed().getValue();

    elevatorLeadMotor =
        new TalonFX(
            Constants.SubsystemConstants.elevator.leadMotorID,
            Constants.SubsystemConstants.elevator.kCanbus);
    elevatorFollowMotor =
        new TalonFX(
            Constants.SubsystemConstants.elevator.followMotorID,
            Constants.SubsystemConstants.elevator.kCanbus);

    // Set the follow motor to follow the lead motor
    elevatorFollowMotor.setControl(new Follower(elevatorLeadMotor.getDeviceID(), false));
  }

  // Method to set the target position using Motion Magic
  public void setElevatorPosition(double targetPositionInMotorTicks) {
    if (targetPositionInMotorTicks == 0) {
      elevatorLeadMotor.setControl(mm_request.withPosition(0).withFeedForward(-0.1));
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
    return Math.abs(elevatorLeadMotor.getPosition().getValueAsDouble() - targetPosition) < .25;
  }

  public double getPosition() {
    return elevatorLeadMotor.getPosition().getValueAsDouble();
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

  public Command elevatorBellowZeroCommand() {
    return new RunCommand(() -> setElevatorPosition(-2), this);
  }

  public Command setElevatorToRestCommand() {
    return elevatorBellowZeroCommand()
        .until(LimitPressed)
        .andThen(runOnce(() -> elevatorLeadMotor.stopMotor()));
  }
}
