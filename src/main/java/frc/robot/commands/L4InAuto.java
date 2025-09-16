package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class L4InAuto extends Command {
  private final Elevator elevator;

  public L4InAuto(Elevator elevator) {
    this.elevator = elevator;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    elevator.setElevatorPosition(elevator.L4_inMotorRotations);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return elevator.isAtAcceptablePosition(elevator.L4_inMotorRotations);
  }
}
