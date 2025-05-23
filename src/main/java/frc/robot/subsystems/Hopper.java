// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
  private final SparkMax hopperMotor;

  public Hopper() {
    hopperMotor = new SparkMax(62, SparkMax.MotorType.kBrushless);
  }

  public void setIntakeSpeed(double velocity) {
    hopperMotor.set(velocity);
  }

  public Command holdHopper_Command(double velocity) {
    return this.run(() -> setIntakeSpeed(velocity));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
