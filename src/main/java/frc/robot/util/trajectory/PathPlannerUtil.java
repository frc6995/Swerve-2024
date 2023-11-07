package frc.robot.util.trajectory;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Workaround for broken impl in pathplannerlib. remove before kickoff.
 */
public class PathPlannerUtil {
    public static List<Translation2d> bezierFromPoses(List<Pose2d> poses) {
    if (poses.size() < 2) {
      throw new IllegalArgumentException("Not enough poses");
    }

    List<Translation2d> bezierPoints = new ArrayList<>();

    // First pose
    bezierPoints.add(poses.get(0).getTranslation());
    bezierPoints.add(
        poses.get(0).getTranslation().plus(new Translation2d(
            poses.get(0).getTranslation().getDistance(poses.get(1).getTranslation()) / 3.0,
            poses.get(0).getRotation())));

    // Middle poses
    for (int i = 1; i < poses.size() - 2; i++) {
      // Prev control
      bezierPoints.add(
          poses.get(i).getTranslation().plus(new Translation2d(
              poses.get(i).getTranslation().getDistance(poses.get(i - 1).getTranslation()) / 3.0,
              poses.get(i).getRotation().plus(Rotation2d.fromDegrees(180)))));
      // Anchor
      bezierPoints.add(poses.get(i).getTranslation());
      // Next control
      bezierPoints.add(
          poses.get(i).getTranslation().plus(new Translation2d(
              poses.get(i).getTranslation().getDistance(poses.get(i + 1).getTranslation()) / 3.0,
              poses.get(i).getRotation())));
    }

    // Last pose
    bezierPoints.add(
        poses.get(poses.size() - 1).getTranslation().plus(
        new Translation2d(
            poses
                    .get(poses.size() - 1)
                    .getTranslation()
                    .getDistance(poses.get(poses.size() - 2).getTranslation())
                / 3.0,
            poses.get(poses.size() - 1).getRotation().plus(Rotation2d.fromDegrees(180)))));
    bezierPoints.add(poses.get(poses.size() - 1).getTranslation());

    return bezierPoints;
  }

}
