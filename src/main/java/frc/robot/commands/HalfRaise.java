package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MailBox;
import frc.robot.subsystems.elevator.Elevator;

public class HalfRaise extends Command {
  private final Elevator elevator;
  private final MailBox mailbox;

  public HalfRaise(Elevator elevator, MailBox mailbox) {
    this.elevator = elevator;
    this.mailbox = mailbox;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean hasCoral = mailbox.somethingInIntake();
    if (hasCoral) {
      elevator.setElevatorPosition(elevator.L4_inMotorRotations / 2, 150, 100, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.isAtAcceptablePosition(elevator.L4_inMotorRotations / 2);
  }
}
