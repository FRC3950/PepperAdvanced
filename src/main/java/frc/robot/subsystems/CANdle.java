// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANdle extends SubsystemBase {
  /** Creates a new CANdle. */
  CANdle candle = new CANdle(SubsystemConstants.lighting.candle);

  public CANdle() {
    CANdleConfiguration lightingConfig = new CANdleConfiguration();
    lightingConfig.stripType = LEDStripType.RGB;
    lightingConfig.brightnessScalar = 1.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
