// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MailBox;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class intakeOnlyWhileEmpty extends Command {
  private final MailBox mailbox;
  /** Creates a new intakeOnlyWhileEmpty. */
  public intakeOnlyWhileEmpty(MailBox mailbox) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mailbox = mailbox;
    addRequirements(mailbox);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mailbox.setIntakeMotor(0);
    // mailbox.MailBox_SetToIntakePosition_Command();
    mailbox.setAngleMotor(11.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mailbox.setIntakeMotor(.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mailbox.setIntakeMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("command finished");
    return mailbox.nothingInIntake();
    // mailbox.somethingInIntake();
  }
}
