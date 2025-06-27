package frc.robot.subsystems.algae;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.S2StateValue;

public class Algae extends SubsystemBase {
  public boolean hasAlgae = false;
  CANdi candi = new CANdi(17, "CANivore");
  TalonFX algaeMotor = new TalonFX(Constants.SubsystemConstants.algae.algaeMotor, Constants.SubsystemConstants.algae.kCanbus);

  public Algae() {
    
  }

  double raw = candi.getS2State().getValue().value;

  @Override
  public void periodic() {
    
    if (raw == 2) {
      hasAlgae = true;
    } else {
      hasAlgae = false;
    }
  }
}
