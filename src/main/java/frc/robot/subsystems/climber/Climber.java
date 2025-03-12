// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  private final TalonFX climberMotor =
      new TalonFX(
          Constants.SubsystemConstants.climber.climberMotor,
          Constants.SubsystemConstants.climber.kCanbus);
  private final DynamicMotionMagicVoltage mm_request = new DynamicMotionMagicVoltage(0, 20, 20, 0);
  private final double restPosition = 0;
  private final double climbPosition = 20; // TBD

  /** Creates a new Climber. */
  public Climber() {}

  public double currentPosition() {
    return climberMotor.getPosition().getValueAsDouble();
  }

  // TODO test bounds
  public boolean isAtAcceptablePosition(double targetPosition) {
    return Math.abs(currentPosition() - targetPosition) < 1;
  }

  public void goToClimbPosition() {
    mm_request.Velocity = 10;
    mm_request.Acceleration = 4;
    mm_request.Jerk = 0;

    climberMotor.setControl(mm_request.withPosition(climbPosition));
  }

  public void goToRestPosition() {
    mm_request.Velocity = 10;
    mm_request.Acceleration = 4;
    mm_request.Jerk = 0;

    climberMotor.setControl(mm_request.withPosition(restPosition));
  }

  public void bringInTheClimb() {
    mm_request.Velocity = 10;
    mm_request.Acceleration = 4;
    mm_request.Jerk = 0;

    climberMotor.setControl(mm_request.withPosition(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
