package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MailBox;

public class OutakeInAuto extends Command {

  private final MailBox mailbox;

  public OutakeInAuto(MailBox mailbox) {
    this.mailbox = mailbox;
    addRequirements(mailbox);
  }

  @Override
  public void initialize() {
    mailbox.setIntakeMotor(mailbox.outakeSpeed);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    mailbox.setIntakeMotor(0);
  }

  @Override
  public boolean isFinished() {
    return mailbox.nothingInIntake();
  }
}
