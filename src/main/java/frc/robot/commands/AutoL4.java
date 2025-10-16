package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MailBox;
import frc.robot.subsystems.elevator.Elevator;

public class AutoL4 extends Command {
  private final Elevator elevator;
  private final MailBox mailbox;

  public AutoL4(Elevator elevator, MailBox mailbox) {
    this.elevator = elevator;
    this.mailbox = mailbox;
    addRequirements(elevator, mailbox);
  }

  @Override
  public void initialize() {
    elevator.setElevatorPosition(elevator.L4_inMotorRotations);
  }

  @Override
  public void execute() {
    double currentPos = elevator.getPosition();
    if (currentPos >= elevator.L4_inMotorRotations * 0.975) {
      mailbox.setOutakeMotor(mailbox.outakeSpeed);
    }
  }

  @Override
  public boolean isFinished() {
    return mailbox.nothingInIntake();
  }

  @Override
  public void end(boolean interrupted) {}
}
