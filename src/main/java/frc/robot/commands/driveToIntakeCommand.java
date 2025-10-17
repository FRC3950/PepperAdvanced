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

public class driveToIntakeCommand extends Command {
  private final Drive drive;
  private final DoubleSupplier driveStickMoved;
  private final PathConstraints constraints =
      new PathConstraints(3.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(540));
  private static final AprilTagFieldLayout aprilTagLayoutForAutoDrive =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  private final int[] aprilTagIdsForScoring = new int[] {1, 2, 12, 13};
  private final Pose2d[] poseForScoringIDs;
  private Command pathCommand;
  public static boolean goingIntake;

  public driveToIntakeCommand(Drive drive, DoubleSupplier driveStickMoved) {
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

  @Override
  public void initialize() {
    // For each pose, find the closest one to the current pose
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
    Pose2d targetPose = closestPose.transformBy(new Transform2d(.5, .0, new Rotation2d(0)));
    // Compose the path following command (do NOT schedule!)
    pathCommand = AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
    pathCommand.initialize();
  }

  @Override
  public void execute() {
    if (pathCommand != null) {
      pathCommand.execute();
      goingIntake = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (pathCommand != null) {
      pathCommand.end(interrupted);
      goingIntake = false;
    }
  }

  @Override
  public boolean isFinished() {
    return pathCommand == null
        || pathCommand.isFinished()
        || Math.abs(driveStickMoved.getAsDouble()) > 0.5; // Stop if operator moves drive stick
  }
}
