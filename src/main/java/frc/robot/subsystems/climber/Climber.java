// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  private final TalonFX climberMotor =
      new TalonFX(
          Constants.SubsystemConstants.climber.climberMotor,
          Constants.SubsystemConstants.climber.kCanbus);
  private final MotionMagicVoltage mm_request = new MotionMagicVoltage(0);

  /** Creates a new Climber. */
  public Climber() {}

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

    climberMotor.setControl(mm_request.withPosition(60));
  }

  public void bringInTheClimb() {

    climberMotor.setControl(mm_request.withPosition(121));
  }

  public void goBackToRest() {

    climberMotor.setControl(mm_request.withPosition(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
