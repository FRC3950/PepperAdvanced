// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MailBox extends SubsystemBase {

  private final SparkMax intakeMotor;
  public final double intakeSpeed = -0.15;
  public final double outakeSpeedL1 = -0.0845;
  public final double outakeSpeed = -0.15;
  public final double sourceAngleEncoder = 11.5;

  // 60:1 gear ratio - 60 spins of the motor = 1 spin of the output shaft
  // I'm assumming we intake at 35 degrees from the horizontal
  // 35/360 * 60

  /** Creates a new MailBox. */
  public MailBox() {

    intakeMotor =
        new SparkMax(
            Constants.SubsystemConstants.mailbox.intakeMotor, SparkMax.MotorType.kBrushless);
  }

  public void setIntakeMotor(double speed) {

    intakeMotor.set(speed);
  }

  public void setOutakeMotor(double speed) {

    intakeMotor.set(speed);
  }

  public void setOutakeMotorL1(double speed) {
    intakeMotor.set(speed);
  }

  public boolean nothingInIntake() {
    return !intakeMotor.getForwardLimitSwitch().isPressed();
  }

  public boolean somethingInIntake() {
    return intakeMotor.getForwardLimitSwitch().isPressed();
  }

  @Override
  public void periodic() {
    //  // This method will be called once per scheduler run
    // System.out.println(intakeMotor.getForwardLimitSwitch().isPressed());
    // false means we are clogged
  }

  ///////////////////////////////////////////

  public Command MailBox_Intake_Command(double intakeSpeed) {
    return this.runOnce(() -> setIntakeMotor(intakeSpeed));
  }

  public Command MailBox_Outake_Command(double outakeSpeed) {
    return this.runOnce(() -> setOutakeMotor(outakeSpeed));
  }

  public Command MailBox_Outake_L1_Command(double outakeSpeedL1) {
    return this.runOnce(() -> setOutakeMotorL1(outakeSpeedL1));
  }

  public Command MailBox_StopIntake_Command() {
    return this.runOnce(() -> setIntakeMotor(0));
  }

  public Command start_stop_IntakeCommand() {
    return this.startEnd(() -> setIntakeMotor(intakeSpeed), () -> setIntakeMotor(0));
  }
}
