package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MailBox;

public class IntakeInAuto extends Command {

  private final MailBox mailbox;

  public IntakeInAuto(MailBox mailbox) {
    this.mailbox = mailbox;
    addRequirements(mailbox);
  }

  @Override
  public void initialize() {
    mailbox.setIntakeMotor(mailbox.intakeSpeed);
  }

  @Override
  public void execute() {
    mailbox.setIntakeMotor(mailbox.intakeSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    mailbox.setIntakeMotor(0);
  }

  @Override
  public boolean isFinished() {
    return mailbox.somethingInIntake();
  }
}
