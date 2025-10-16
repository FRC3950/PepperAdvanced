package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.MailBox;
import frc.robot.subsystems.elevator.Elevator;

public class ScoreCommand extends Command {
  private final Elevator elevator;
  private final MailBox mailbox;
  private final double targetHeight;
  private final double triggerPercent;
  private final double mailboxSpeed;
  private boolean mailboxStarted = false;
  private boolean scored = false;

  public ScoreCommand(
      Elevator elevator,
      MailBox mailbox,
      double targetHeight,
      double triggerPercent,
      double mailboxSpeed) {
    this.elevator = elevator;
    this.mailbox = mailbox;
    this.targetHeight = targetHeight;
    this.triggerPercent = triggerPercent;
    this.mailboxSpeed = mailboxSpeed;
    addRequirements(elevator, mailbox);
  }

  @Override
  public void initialize() {
    elevator.setElevatorPosition(targetHeight);
    mailboxStarted = false;
    scored = false;
  }

  @Override
  public void execute() {
    double currentPos = elevator.getPosition();
    // Start mailbox motor when at threshold percentage of height
    if (!mailboxStarted && currentPos >= targetHeight * triggerPercent) {
      mailbox.setOutakeMotor(mailboxSpeed);
      mailboxStarted = true;
    }

    // Check for scoring condition (either elevator at position or sensor - example:
    // mailbox.nothingInIntake())
    if (mailboxStarted && mailbox.nothingInIntake()) {
      scored = true;
      CommandScheduler.getInstance().schedule(elevator.setElevatorToRestCommand()); // Go back down
    }
  }

  @Override
  public boolean isFinished() {
    // Finished when elevator goes back to zero AND scoring detected
    return scored && elevator.isAtAcceptablePosition(0);
  }

  @Override
  public void end(boolean interrupted) {}
}
