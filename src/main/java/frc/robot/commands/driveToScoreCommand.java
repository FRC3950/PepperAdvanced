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

public class driveToScoreCommand extends Command {
  private final Drive drive;
  private final LightsSubsystem lights;
  private final String direction;
  private Command pathCommand;
  public static boolean aligning;

  public driveToScoreCommand(Drive drive, LightsSubsystem lights, String direction) {
    this.drive = drive;
    this.lights = lights;
    this.direction = direction;
    addRequirements(drive, lights);
  }

  @Override
  public void initialize() {
    PathConstraints constraints =
        new PathConstraints(3.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(540));
    AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    int[] scoringTagIDs = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
    Pose2d[] scorePoses = new Pose2d[scoringTagIDs.length];
    for (int i = 0; i < scoringTagIDs.length; i++) {
      scorePoses[i] = aprilTagLayout.getTagPose(scoringTagIDs[i]).orElse(new Pose3d()).toPose2d();
    }

    // Find closest pose
    Pose2d currentPose = drive.getPose();
    Pose2d closestPose = scorePoses[0];
    double minDistance = Double.MAX_VALUE;
    for (Pose2d pose : scorePoses) {
      double distance = pose.getTranslation().getDistance(currentPose.getTranslation());
      if (distance < minDistance) {
        minDistance = distance;
        closestPose = pose;
      }
    }

    // Offset based on direction
    Pose2d targetPose = closestPose;
    if ("left".equals(direction)) {
      targetPose =
          targetPose.transformBy(
              new Transform2d(.5, -.1645, new Rotation2d().rotateBy(new Rotation2d(Math.PI))));
    } else {
      targetPose =
          targetPose.transformBy(
              new Transform2d(.5, .1645, new Rotation2d().rotateBy(new Rotation2d(Math.PI))));
    }

    // Compose the actual movement command (DO NOT schedule!)
    pathCommand =
        AutoBuilder.pathfindToPose(targetPose, constraints, 0.0)
            .beforeStarting(
                () -> {
                  /*lights.setLEDOverride(true, AnimationType.Strobe);*/
                })
            .finallyDo(
                (interrupted) -> {
                  /*lights.setLEDOverride(false, null);*/
                });

    // Initialize the internal command
    pathCommand.initialize();
  }

  @Override
  public void execute() {
    if (pathCommand != null) {
      pathCommand.execute();
      aligning = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (pathCommand != null) {
      pathCommand.end(interrupted);
      aligning = false;
    }
  }

  @Override
  public boolean isFinished() {
    return pathCommand == null || pathCommand.isFinished();
  }
}
