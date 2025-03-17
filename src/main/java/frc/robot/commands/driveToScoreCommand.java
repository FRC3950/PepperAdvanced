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

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class driveToScoreCommand extends Command {
  private final Drive drive;
  private final String direction;
  private Pose2d targetPose;
  private Command pathCommand;
  private Pose2d targetPoseToUseInAutoNavigate;

  public PathConstraints constraints =
      new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayoutForAutoDrive =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  int[] aprilTagIdsForScoring = new int[] {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
  Pose2d[] poseForScoringIDs;

  /** Creates a new driveToScoreCommand. */
  public driveToScoreCommand(Drive drive, String direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.direction = direction;
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

    System.out.println("Starting Drive!");
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
    if (direction.equals("left")) {
      targetPose =
          targetPose.transformBy(
              new Transform2d(.5, -.145, new Rotation2d().rotateBy(new Rotation2d(Math.PI))));
    } else {
      targetPose =
          targetPose.transformBy(
              new Transform2d(.5, .145, new Rotation2d().rotateBy(new Rotation2d(Math.PI))));
    }

    pathCommand = AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
    pathCommand
        .schedule(); // Not in love with this - Spent way to much timme trying to lauch command
    // straight from Autobuilder, but with AutoBuilder being static it was being
    // weird. This is a hack to get it to work. I hate that we are
    // assigning a command in a command, and the schelduing it. Not sure how the drive subystem
    // doesn't collided, but yolo.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    System.out.println("DriveToPoseCommand finished.");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathCommand == null || pathCommand.isFinished(); // Stop when PathPlanner finishes
  }
}
