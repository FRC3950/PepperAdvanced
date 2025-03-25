// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Climber extends SubsystemBase {

  Slot0Configs slot0 = new Slot0Configs();
  Slot1Configs slot1 = new Slot1Configs();
  
  @Override
  public void addChild(String name, Sendable child) {
    super.addChild(name, child);
  }

  private final TalonFX climberMotor =
      new TalonFX(
          Constants.SubsystemConstants.climber.climberMotor,
          Constants.SubsystemConstants.climber.kCanbus);
  private final MotionMagicVoltage mm_request = new MotionMagicVoltage(0);

  /** Creates a new Climber. */
  public Climber() {
    slot0.kP = 0.1;
    slot0.kS = 0.1;
    slot0.kV = 0.12;

    slot1.kP = 0.2;
    slot1.kS = 0.1;
    slot1.kV = 0.12;

    climberMotor.getConfigurator().apply(slot0);

    SmartDashboard.putNumber("Climber Init Pos", 2.8);
  }

  public double currentPosition() {
    return climberMotor.getPosition().getValueAsDouble();
  }

  public boolean isReadyToClimb() {
    return currentPosition() > 50;
  }

  // TODO test bounds
  public boolean isAtAcceptablePosition(double targetPosition) {
    return Math.abs(currentPosition() - targetPosition) < 1;
  }

  public void goToClimbInitPosition() {

    climberMotor.setControl(mm_request.withPosition(85));
  }

  public void bringInTheClimb() {

    climberMotor.getConfigurator().apply(slot1);

    climberMotor.setControl(mm_request.withPosition(252));
  }

  public void goBackToRestSmartDashboard() {
    climberMotor.setControl(mm_request.withPosition(0));
  }

  public void goBackToRest() {

    climberMotor.setControl(mm_request.withPosition(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
