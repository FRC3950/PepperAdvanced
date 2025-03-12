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
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class driveToIntakeCommand extends Command {
  private final Drive drive;
  private final DoubleSupplier driveStickMoved;
  private Pose2d targetPose;
  private Command pathCommand;
  private Pose2d targetPoseToUseInAutoNavigate;

  public PathConstraints constraints =
      new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayoutForAutoDrive =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  int[] aprilTagIdsForScoring = new int[] {1, 2, 12, 13};
  Pose2d[] poseForScoringIDs;

  /** Creates a new driveToScoreCommand. */
  public driveToIntakeCommand(Drive drive, DoubleSupplier driveStickMoved) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.driveStickMoved = driveStickMoved;
    addRequirements(drive);

    poseForScoringIDs = new Pose2d[aprilTagIdsForScoring.length];

    for (int i = 0; i < aprilTagIdsForScoring.length; i++) {
      poseForScoringIDs[i] =
          aprilTagLayoutForAutoDrive
              .getTagPose(aprilTagIdsForScoring[i])
              .orElse(new Pose3d())
              .toPose2d();
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    System.out.println("Starting Drive");
    // for each pose in poseForScoringIDs, find the closest one to the current pose
    Pose2d currentPose = drive.getPose();
    double minDistance = 1000.0;
    Pose2d closestPose = new Pose2d();

    for (Pose2d pose : poseForScoringIDs) {

      double distance = pose.getTranslation().getDistance(currentPose.getTranslation());
      if (distance < minDistance) {
        minDistance = distance;
        closestPose = pose;
      }
    }

    targetPose = closestPose;

    targetPose = targetPose.transformBy(new Transform2d(.5, .0, new Rotation2d(Math.PI)));

    pathCommand = AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
    pathCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    System.out.println("Drive To Intake Finsihed");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathCommand == null
        || pathCommand.isFinished()
        || Math.abs(driveStickMoved.getAsDouble()) > 0.5; // Stop when PathPlanner finishes
  }
}
