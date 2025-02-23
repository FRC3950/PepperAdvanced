// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private final TalonFX elevatorLeadMotor;
  private final TalonFX elevatorFollowMotor;
  private final DynamicMotionMagicVoltage mm_request =
      new DynamicMotionMagicVoltage(0, 60, 120, 1200);

  private final double L1_inMotorRotations = 10;
  private final double L2_inMotorRotations = 20;
  private final double L3_inMotorRotations = 25;
  private final double L4_inMotorRotations = 35;

  private final double k_intakeHeightInMotorRotations = 50;

  private final Mechanism2d mech = new Mechanism2d(3, 3);
  private final MechanismRoot2d root = mech.getRoot("elevator", 1.5, 0);
  private final MechanismLigament2d v_elevator;

  private double currentPosSim = 0;

  /** Creates a new Elevator. */
  public Elevator() {
    v_elevator = root.append(new MechanismLigament2d("elevator", 1, 90));
    SmartDashboard.putData("Mech2d", mech);

    // 3:1 gear ratio - 3 spins of the motor = 1 spin of the output shaft
    // REally loose assumptin that 1 spin is 6.25 inches
    // 110 / 6.25 = 17.6 rotations
    // 17.6 * 3 = 52.8
    // Let's assume 50 Rotations is the top of the elevator

    elevatorLeadMotor =
        new TalonFX(
            Constants.SubsystemConstants.elevator.leadMotorID,
            Constants.SubsystemConstants.elevator.kCanbus);
    elevatorFollowMotor =
        new TalonFX(
            Constants.SubsystemConstants.elevator.followMotorID,
            Constants.SubsystemConstants.elevator.kCanbus);

    // Sim

    var talonFXConfigs = new TalonFXConfiguration();

    // Set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 1.2; // A position error of 10 rotations results in 12 V output
    slot0Configs.kI = 0; // No output for integrated error
    slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    elevatorLeadMotor.getConfigurator().apply(talonFXConfigs);

    // Set the follow motor to follow the lead motor
    elevatorFollowMotor.setControl(new Follower(elevatorLeadMotor.getDeviceID(), false));
  }

  // Method to set the target position using Motion Magic
  public void setElevatorPosition(double targetPositionInMotorTicks) {

    currentPosSim = targetPositionInMotorTicks;
    if (targetPositionInMotorTicks > elevatorLeadMotor.getPosition().getValueAsDouble()) {

      mm_request.Velocity = 60;
      mm_request.Acceleration = 120;
      mm_request.Jerk = 1200;
    } else {
      mm_request.Velocity = 30;
      mm_request.Acceleration = 60;
      mm_request.Jerk = 600;
    }

    elevatorLeadMotor.setControl(mm_request.withPosition(targetPositionInMotorTicks));
  }

  public void setElevatorPosition(
      double targetPositionInMotorTicks, double velocity, double acceleration, double jerk) {
    currentPosSim = targetPositionInMotorTicks;
    mm_request.Velocity = velocity;
    mm_request.Acceleration = acceleration;
    mm_request.Jerk = jerk;

    elevatorLeadMotor.setControl(mm_request.withPosition(targetPositionInMotorTicks));
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
    return this.setElevatorPositionCommand(L1_inMotorRotations);
  }

  public Command setElevatorToL2Command() {
    return this.setElevatorPositionCommand(L2_inMotorRotations);
  }

  public Command setElevatorToL3Command() {
    return this.setElevatorPositionCommand(L3_inMotorRotations);
  }

  public Command setElevatorToL4Command() {
    return this.setElevatorPositionCommand(L4_inMotorRotations);
  }

  public Command setElevatorToRestCommand() {
    return this.setElevatorPositionCommand(0);
  }
}
