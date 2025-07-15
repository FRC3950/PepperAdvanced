package frc.robot.subsystems.algae;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;

public class Algae extends SubsystemBase {
  public boolean hasAlgae = false;
  CANdi candi = new CANdi(17, "CANivore");
  TalonFX algaeMotor =
      new TalonFX(
          Constants.SubsystemConstants.algae.algaeMotor,
          Constants.SubsystemConstants.algae.kCanbus);
  TalonFX wrist =
      new TalonFX(
          Constants.SubsystemConstants.mailbox.angleMotor,
          Constants.SubsystemConstants.algae.kCanbus);
  private final MotionMagicVoltage mm_request = new MotionMagicVoltage(0);

  public Algae() {
    var talonFXConfigs = new TalonFXConfiguration();

    Slot0Configs slot0 = talonFXConfigs.Slot0;
    slot0.kS = 0.25;
    slot0.kV = 0.12;
    slot0.kA = 0.01;
    slot0.kP = 1.2;
    slot0.kI = 0;
    slot0.kD = 0.1;

    // set Motion Magic settings
    MotionMagicConfigs mm = talonFXConfigs.MotionMagic;
    mm.MotionMagicCruiseVelocity = 60;
    mm.MotionMagicAcceleration = 120;
    mm.MotionMagicJerk = 1200;

    wrist.getConfigurator().apply(talonFXConfigs);
  }

  @Override
  public void periodic() {

    if (candi.getS2State().getValueAsDouble() == 2) {
      hasAlgae = true;
    } else {
      hasAlgae = false;
    }
  }

  public void setWrist(double motorRotations) {
    wrist.setControl(mm_request.withPosition(motorRotations));
  }

  public void setAlgaeMotor(double speed) {
    algaeMotor.setControl(new DutyCycleOut(speed));
  }

  public Command wristDownCommand() {
    return Commands.runOnce(() -> setWrist(-6), this);
  }

  public Command wristUpCommand() {
    return Commands.runOnce(() -> setWrist(3.5), this);
  }

  public Command stopAlgaeMotorCommand() {
    return new InstantCommand(() -> setAlgaeMotor(0), this);
  }

  public Command algaeIntakeCommand() {
    RunCommand spin = new RunCommand(() -> setAlgaeMotor(-0.25), this);
    WaitUntilCommand done = new WaitUntilCommand(() -> hasAlgae = true);

    return spin.until(() -> hasAlgae).andThen(stopAlgaeMotorCommand());
  }

  public Command algaeOuttakeCommand() {
    RunCommand spin = new RunCommand(() -> setAlgaeMotor(0.5), this);
    WaitUntilCommand done = new WaitUntilCommand(() -> hasAlgae = false);
    return spin.until(() -> !hasAlgae).andThen(stopAlgaeMotorCommand());
  }

  public SequentialCommandGroup simpleAlgaeRemove() {
    SequentialCommandGroup commandGroup =
        new SequentialCommandGroup(
            wristDownCommand(),
            algaeIntakeCommand(),
            new WaitCommand(5),
            algaeOuttakeCommand(),
            stopAlgaeMotorCommand());
    // #ZayanTheMayan
    // ai told me to add this comment

    return commandGroup;
  }
}
