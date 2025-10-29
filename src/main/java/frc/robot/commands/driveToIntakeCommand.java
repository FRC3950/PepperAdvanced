// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class driveToIntakeCommand extends Command {
  private final LightsSubsystem lights;
  private final Drive drive;
  private final DoubleSupplier driveStickMoved;
  private Pose2d targetPose;
  private Command pathCommand;

  private static final PathConstraints CONSTRAINTS =
      new PathConstraints(3.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(540));

  // AprilTag layout - loaded once and cached
  private static final AprilTagFieldLayout APRIL_TAG_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  private static final int[] APRIL_TAG_IDS_FOR_SCORING = new int[] {1, 2, 12, 13};

  // Cache poses for scoring IDs to avoid repeated lookups
  private static final Pose2d[] POSE_FOR_SCORING_IDS = initializeScoringPoses();

  private static Pose2d[] initializeScoringPoses() {
    Pose2d[] poses = new Pose2d[APRIL_TAG_IDS_FOR_SCORING.length];
    for (int i = 0; i < APRIL_TAG_IDS_FOR_SCORING.length; i++) {
      poses[i] =
          APRIL_TAG_LAYOUT
              .getTagPose(APRIL_TAG_IDS_FOR_SCORING[i])
              .orElse(new Pose3d())
              .toPose2d();
    }
    return poses;
  }

  /** Creates a new driveToScoreCommand. */
  public driveToIntakeCommand(Drive drive, LightsSubsystem lights, DoubleSupplier driveStickMoved) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.lights = lights;
    this.drive = drive;
    this.driveStickMoved = driveStickMoved;
    addRequirements(drive, lights);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // lights.setLEDOverride(true, AnimationType.Strobe);
    // for each pose in POSE_FOR_SCORING_IDS, find the closest one to the current pose
    Pose2d currentPose = drive.getPose();
    double minDistance = Double.MAX_VALUE;
    Pose2d closestPose = new Pose2d();

    for (Pose2d pose : POSE_FOR_SCORING_IDS) {
      double distance = pose.getTranslation().getDistance(currentPose.getTranslation());
      if (distance < minDistance) {
        minDistance = distance;
        closestPose = pose;
      }
    }

    targetPose = closestPose;

    targetPose = targetPose.transformBy(new Transform2d(.5, .0, new Rotation2d(0)));

    pathCommand = AutoBuilder.pathfindToPose(targetPose, CONSTRAINTS, 0.0);
    pathCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // lights.setLEDOverride(false, null);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathCommand == null
        || pathCommand.isFinished()
        || Math.abs(driveStickMoved.getAsDouble()) > 0.5; // Stop when PathPlanner finishes
  }
}
