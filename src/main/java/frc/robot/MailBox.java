// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MailBox extends SubsystemBase {

  private final SparkMax intakeMotor;
  private final TalonFX angleMotor;
  private final double intakeSpeed = 0.5;
  private final double outakeSpeed = -0.25;
  public final double sourceAngleEncoder = 11.5;

  // 60:1 gear ratio - 60 spins of the motor = 1 spin of the output shaft
  // I'm assumming we intake at 35 degrees from the horizontal
  // 35/360 * 60
  private final double k_intakeAngleRotations = 5.8333333333333333333333333333333;

  private final MotionMagicVoltage mm_request = new MotionMagicVoltage(0);
  private double voltageToHoldHorizontal = 0;

  /** Creates a new MailBox. */
  public MailBox() {

    intakeMotor =
        new SparkMax(
            Constants.SubsystemConstants.mailbox.intakeMotor, SparkMax.MotorType.kBrushless);

    angleMotor = new TalonFX(Constants.SubsystemConstants.mailbox.angleMotor, "CANivore");

    var talonFXConfigs = new TalonFXConfiguration();

    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP =
        1.2; // A position error of 10 rotations results in 12 V output  ////Starting low for safety
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 60; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration =
        120; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1200; // Target jerk of 1600 rps/s/s (0.1 seconds)

    // angleMotor.getConfigurator().apply(talonFXConfigs);
  }

  public void setIntakeMotor(double speed) {

    intakeMotor.set(speed);
  }

  public void setOutakeMotor(double speed) {

    intakeMotor.set(speed);
  }

  public boolean nothingInIntake() {
    return intakeMotor.getForwardLimitSwitch().isPressed();
  }

  public boolean somethingInIntake() {
    return !intakeMotor.getForwardLimitSwitch().isPressed();
  }

  public void setAngleMotor(double motorRotations) {
    angleMotor.setControl(mm_request.withPosition(motorRotations));

    // This allows a Sin curve to be used to calculate the voltage needed to hold the mailbox at a
    // certain angle

    // var angle = motorRotations / 60 * 360;
    // var radians = Math.toRadians(angle);
    // var output = Math.sin(radians);
    // var scaledOutput = output * voltageToHoldHorizontal;
    // angleMotor.setControl(mm_request.withPosition(motorRotations).withFeedForward(scaledOutput));

    // ----------------------Just words----------------------

    // 35 degrees to radians is 0.6108652381980153
    // 0.6108652381980153 in Sin is 0.5720614028176847
    // 90 degrees to radians is 1.5707963267948966
    // 1.5707963267948966 in Sin is 1

    // ----------------------Just words----------------------

  }

  @Override
  public void periodic() {
    //  // This method will be called once per scheduler run
    // System.out.println(intakeMotor.getForwardLimitSwitch().isPressed());
    // false means we are clogged
    SmartDashboard.putNumber("angle motor ticks ", angleMotor.getPosition().getValueAsDouble());
  }

  public Command MailBox_SetToIntakePosition_Command() {
    return this.runOnce(() -> setAngleMotor(11.5));
  }

  public Command MailBox_SetToL2L3_Position_Command() {
    return this.runOnce(() -> setAngleMotor(4));
  }

  public Command MailBox_SetL4_Position_Command() {
    return this.runOnce(() -> setAngleMotor(5));
  }

  public Command MailBox_SetToHorizontalPosition_Command() {
    return this.runOnce(() -> setAngleMotor(0));
  }

  public Command incrementBy6Degress_Command() {
    return this.runOnce(() -> setAngleMotor(angleMotor.getPosition().getValueAsDouble() + .2));
  }

  public Command decrementBy6Degress_Command() {
    return this.runOnce(() -> setAngleMotor(angleMotor.getPosition().getValueAsDouble() - .2));
  }

  ///////////////////////////////////////////

  public Command MailBox_Intake_Command(double intakeSpeed) {
    return this.runOnce(() -> setIntakeMotor(intakeSpeed));
  }

  public Command MailBox_Outake_Command(double outakeSpeed) {
    return this.runOnce(() -> setOutakeMotor(outakeSpeed));
  }

  public Command MailBox_StopIntake_Command() {
    return this.runOnce(() -> setIntakeMotor(0));
  }
}
