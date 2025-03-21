// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
  private final Talon hopperMotor;
  private boolean isHolding = true;

  public Hopper() {
    hopperMotor = new Talon(0);
    hopperMotor.set(.25);
  }

  public void setIntakeSpeed(double velocity) {
    hopperMotor.set(velocity);
  }

  public Command holdHopper_Command(double velocity) {
    return this.run(() -> setIntakeSpeed(velocity));
  }

  public void changeHoldingState() {
    isHolding = !isHolding;
  }

  public Command changeHoldingState_Command() {
    return this.runOnce(() -> changeHoldingState());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
