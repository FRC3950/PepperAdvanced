// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MailBox;
import frc.robot.subsystems.elevator.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class intakeTeleopCommand extends Command {
  private final MailBox mailbox;
  private final Elevator elevator;
  /** Creates a new intake_teleop. */
  public 
  intakeTeleopCommand(MailBox mailbox, Elevator elevator) {

    this.mailbox = mailbox;
    this.elevator = elevator;

    addRequirements(mailbox, elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mailbox.setIntakeMotor(0);
    elevator.setElevatorPosition(0);
  }

  @Override
  public void execute() {

    mailbox.setIntakeMotor(mailbox.intakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    mailbox.setIntakeMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mailbox.somethingInIntake();
  }
}
