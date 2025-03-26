// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeDuringAuto;
import frc.robot.commands.L4DuringAuto;
import frc.robot.commands.OutakeDuringAuto;
import frc.robot.commands.driveToIntakeCommand;
import frc.robot.commands.driveToScoreCommand;
import frc.robot.commands.intakeOnlyWhileEmpty;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private Elevator elevator;
  private Hopper hopper;
  private Climber climber;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final MailBox mailbox;
  private Trigger intakeIsAlwaysOnWhenAtRest;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SmartDashboard.putNumber("seedAngle", 180);
    SmartDashboard.putNumber("seedX", 0);
    SmartDashboard.putNumber("seedY", 0);
    SmartDashboard.putData("command", CommandScheduler.getInstance());

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(camera0Name, drive::getRotation),
                new VisionIOLimelight(camera1Name, drive::getRotation));

        // Don't uncomment
        // vision =
        //     new Vision(
        //         demoDrive::addVisionMeasurement,
        //         new VisionIOPhotonVision(camera0Name, robotToCamera0),
        //         new VisionIOPhotonVision(camera1Name, robotToCamera1));

        elevator = new Elevator();

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));

        elevator = new Elevator();

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        elevator = new Elevator();

        break;
    }
    mailbox = new MailBox();
    hopper = new Hopper();
    climber = new Climber();
    intakeOnlyWhileEmpty intakeCommandforSource = new intakeOnlyWhileEmpty(mailbox);

    intakeIsAlwaysOnWhenAtRest =
        new Trigger(
            () ->
                mailbox.nothingInIntake()
                    && elevator.isAtAcceptablePosition(0)
                    && Robot.isTeleop == true);

    NamedCommands.registerCommand(
        "SlowlyRaise", elevator.setElevatorPositionCommand(elevator.L4_inMotorRotations, 3, 6, 0));

    NamedCommands.registerCommand(
        "L4",
        new L4DuringAuto(elevator)
            .withTimeout(2)); // TODO consider a timeout for if the elevator doesn't reach
    NamedCommands.registerCommand("Rest", elevator.setElevatorToRestCommand());

    NamedCommands.registerCommand(
        "Intake",
        new IntakeDuringAuto(mailbox, elevator)
            .andThen(new WaitUntilCommand(() -> mailbox.somethingInIntake())));
    NamedCommands.registerCommand(
        "Outake",
        new OutakeDuringAuto(mailbox).withTimeout(.6)); // TODO Remove timeout only here for sim

    NamedCommands.registerCommand("Backward", new InstantCommand());

    NamedCommands.registerCommand(
        "AutoLeft", new driveToScoreCommand(drive, "left").withTimeout(2));
    NamedCommands.registerCommand(
        "AutoRight", new driveToScoreCommand(drive, "right").withTimeout(2));
    NamedCommands.registerCommand(
        "AutoSource", new driveToIntakeCommand(drive, () -> 0.0).withTimeout(2));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    // test 3
    // Set up SysId routines

    autoChooser.addOption(
        "seedAuto",
        Commands.runOnce(
            () ->
                drive.setPose(
                    new Pose2d(
                        new Translation2d(
                            SmartDashboard.getNumber("seedX", 6),
                            SmartDashboard.getNumber("seedY", 4)),
                        new Rotation2d(
                            Math.toRadians(SmartDashboard.getNumber("seedAngle", 180))))),
            drive));

    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    controller.a().whileTrue(new driveToIntakeCommand(drive, () -> controller.getLeftY()));

    // // Lock to 0° when A button is held
    // controller
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> new Rotation2d(Math.toRadians(60 + 180))));

    // Reset gyro to 0° when Y button is pressed
    controller
        .y()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // Auto Align Scoring Elements
    controller.leftBumper().onTrue(new driveToScoreCommand(drive, "left"));

    // .onlyWhile(
    //     () ->
    //         Math.abs(controller.getLeftY()) < 0.5
    //             && Math.abs(controller.getLeftX()) < 0.5));

    controller.rightBumper().whileTrue(new driveToScoreCommand(drive, "right"));

    // .onlyWhile(
    //         () ->
    //             Math.abs(controller.getLeftY()) < 0.5
    //                 && Math.abs(controller.getLeftY()) < 0.5));

    // Fine Tune Driving
    controller.pov(90).whileTrue(drive.run(() -> drive.runVelocity(new ChassisSpeeds(0, -0.5, 0))));

    controller.pov(270).whileTrue(drive.run(() -> drive.runVelocity(new ChassisSpeeds(0, 0.5, 0))));

    controller.pov(0).whileTrue(drive.run(() -> drive.runVelocity(new ChassisSpeeds(0.5, 0, 0))));

    controller
        .pov(180)
        .whileTrue(drive.run(() -> drive.runVelocity(new ChassisSpeeds(-0.5, 0, 0))));

    controller
        .leftTrigger(0.5)
        .whileTrue(drive.run(() -> drive.runVelocity(new ChassisSpeeds(0, 0, 0.7))));

    controller
        .rightTrigger(0.5)
        .whileTrue(drive.run(() -> drive.runVelocity(new ChassisSpeeds(0, 0, -0.7))));

    // controller
    //     .start()
    //     .onTrue(
    //         new DeferredCommand(() -> AutoBuilder.pathfindThenFollowPath(null, null),
    // Set.of(drive))
    //             .alongWith(new PrintCommand("Starting Auto")));

    // Operator Controls/////////////////////////

    //   intakeIsAlwaysOnWhenAtRest.whileTrue(mailbox.start_stop_IntakeCommand());

    operator
        .x()
        .onTrue(
            elevator
                .setElevatorToL1Command()
                .andThen(
                    new WaitUntilCommand(
                            () -> elevator.isAtAcceptablePosition(elevator.L1_inMotorRotations))
                        .withTimeout(2.25))
                .andThen(new WaitCommand(0.1))
                .andThen(mailbox.MailBox_Outake_L1_Command(mailbox.outakeSpeedL1))
                .andThen(new WaitUntilCommand(mailbox::nothingInIntake).withTimeout(.5))
                .andThen(elevator.setElevatorToRestCommand())
                .andThen(mailbox.start_stop_IntakeCommand().until(mailbox::somethingInIntake)));

    operator
        .a()
        .onTrue(
            elevator
                .setElevatorToL2Command()
                .andThen(
                    new WaitUntilCommand(
                            () -> elevator.isAtAcceptablePosition(elevator.L2_inMotorRotations))
                        .withTimeout(2.25))
                .andThen(new WaitCommand(0.1))
                .andThen(mailbox.MailBox_Outake_Command(mailbox.outakeSpeed))
                .andThen(new WaitUntilCommand(mailbox::nothingInIntake).withTimeout(.5))
                .andThen(elevator.setElevatorToRestCommand())
                .andThen(mailbox.start_stop_IntakeCommand().until(mailbox::somethingInIntake)));

    operator
        .b()
        .onTrue(
            elevator
                .setElevatorToL3Command()
                .andThen(
                    new WaitUntilCommand(
                            () -> elevator.isAtAcceptablePosition(elevator.L3_inMotorRotations))
                        .withTimeout(2.25))
                .andThen(new WaitCommand(0.1))
                .andThen(mailbox.MailBox_Outake_Command(mailbox.outakeSpeed))
                .andThen(new WaitUntilCommand(mailbox::nothingInIntake).withTimeout(.5))
                .andThen(elevator.setElevatorToRestCommand())
                .andThen(mailbox.start_stop_IntakeCommand().until(mailbox::somethingInIntake)));

    operator
        .y()
        .onTrue(
            elevator
                .setElevatorToL4Command()
                .andThen(
                    new WaitUntilCommand(
                            () -> elevator.isAtAcceptablePosition(elevator.L4_inMotorRotations))
                        .withTimeout(2.25))
                .andThen(new WaitCommand(0.15))
                .andThen(mailbox.MailBox_Outake_Command(mailbox.outakeSpeed))
                .andThen(new WaitUntilCommand(mailbox::nothingInIntake).withTimeout(1))
                .andThen(new WaitCommand(0.1))
                .andThen(elevator.setElevatorToRestCommand())
                .andThen(mailbox.start_stop_IntakeCommand().until(mailbox::somethingInIntake)));

    operator.pov(0).debounce(0.1).onTrue(elevator.incrementElevatorPositionCommand(.5));
    // consider - .debounce(0.1) before on true

    operator.pov(180).debounce(0.1).onTrue(elevator.decrementElevatorPositionCommand(.5));

    operator
        .leftBumper()
        .onTrue(
            elevator
                .setElevatorToRestCommand()
                .andThen(mailbox.start_stop_IntakeCommand().until(mailbox::somethingInIntake)));
    // operator
    //     .leftTrigger(.5)
    //     .onTrue(
    //         new InstantCommand(() -> climber.goToClimbInitPosition(), climber)
    //             .alongWith(hopper.holdHopper_Command(-0.2)));

    // operator
    //     .rightTrigger(.5)
    //     .onTrue(
    //         Commands.runOnce(() -> climber.bringInTheClimb(), climber)
    //             .onlyIf(climber::isReadyToClimb));

    SmartDashboard.putData(
        "Climber Reset Button", new InstantCommand(() -> climber.goBackToRest(), climber));
    //   operator.start().
    // operator.start().onTrue(hopper.holdHopper_Command(-.2));
    // operator.back().onTrue(hopper.holdHopper_Command(0.25));

    // Maniuplator Controls

    // Elevator

    SmartDashboard.putData(
        "Click To Reset Climber to Zero",
        new InstantCommand(() -> climber.goBackToRestSmartDashboard(), climber));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public Command setElevatorToZeroAndZayanTime() {
    return new InstantCommand()
        .andThen(mailbox.start_stop_IntakeCommand().until(mailbox::somethingInIntake));
  }
}
