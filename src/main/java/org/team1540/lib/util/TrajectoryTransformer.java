package org.team1540.lib.util;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.ArrayList;
import java.util.List;


public class TrajectoryTransformer {
    private static final double FIELD_WIDTH_METERS = 8.02;

    public static Pose2d transformPoseForAlliance(Pose2d rawPose, DriverStation.Alliance alliance) {
        if (alliance== DriverStation.Alliance.Blue) {return rawPose;}

        Translation2d transformedTranslation = new Translation2d(rawPose.getX(), FIELD_WIDTH_METERS - rawPose.getY());
        Rotation2d transformedHeading = rawPose.getRotation().times(-1);
        return new Pose2d(transformedTranslation, transformedHeading);

    }
    public static Trajectory.State transformStateForAlliance(
            Trajectory.State state, DriverStation.Alliance alliance) {
        if (alliance == DriverStation.Alliance.Red) {
            // Create a new state so that we don't overwrite the original
            Trajectory.State transformedState = new PathPlannerTrajectory.PathPlannerState();

            Translation2d transformedTranslation =
                    new Translation2d(state.poseMeters.getX(), FIELD_WIDTH_METERS - state.poseMeters.getY());
            Rotation2d transformedHeading = state.poseMeters.getRotation().times(-1);
            transformedState.timeSeconds = state.timeSeconds;
            transformedState.velocityMetersPerSecond = state.velocityMetersPerSecond;
            transformedState.accelerationMetersPerSecondSq = state.accelerationMetersPerSecondSq;
            transformedState.poseMeters = new Pose2d(transformedTranslation, transformedHeading);
            transformedState.curvatureRadPerMeter = -state.curvatureRadPerMeter;

            return transformedState;
        } else {
            return state;
        }
    }

    public static Trajectory transformTrajectoryForAlliance(
            Trajectory trajectory, DriverStation.Alliance alliance) {
        if (alliance == DriverStation.Alliance.Red) {
            List<Trajectory.State> transformedStates = new ArrayList<>();

            for (Trajectory.State s : trajectory.getStates()) {
                transformedStates.add(transformStateForAlliance(s, alliance));
            }

            return new Trajectory(transformedStates);
        } else {
            return trajectory;
        }
    }
}
