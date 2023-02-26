package org.team1540.robot2023.commands.drivetrain;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2023.Constants;
import org.team1540.robot2023.utils.PolePosition;

import static org.team1540.robot2023.Globals.aprilTagLayout;
import static org.team1540.robot2023.Globals.field2d;

class GridDriveCommand extends SequentialCommandGroup {
    private static int getClosestTag(Drivetrain drivetrain) {
        Translation2d currentPose = drivetrain.getPose().getTranslation();
        double mindist = Double.MAX_VALUE;
        int closestTag = -1;
        for (AprilTag tag : aprilTagLayout.getTags()) {
            double distance = tag.pose.toPose2d().getTranslation().getDistance(currentPose);
            if (distance < mindist || closestTag == -1) {
                mindist = distance;
                closestTag = tag.ID;
            }
        }
        return closestTag;
    }
    public GridDriveCommand(Drivetrain drivetrain, PolePosition postiion) {
        this(drivetrain, getClosestTag(drivetrain), postiion);
    }
    public GridDriveCommand(Drivetrain drivetrain, int tag, PolePosition position) {
        Translation2d endPoint = aprilTagLayout.getTagPose(tag).orElseThrow().toPose2d().getTranslation().plus(new Translation2d(Constants.Auto.gridBackoffOffsetMeters, position.offset));
        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
                new PathConstraints(5, 3),
                new PathPoint(drivetrain.getPose().getTranslation(), Rotation2d.fromDegrees(0), drivetrain.getPose().getRotation()), // position, heading(direction of travel), holonomic rotation
//                new PathPoint(layout.getTagPose(tag).orElse(new Pose3d(drivetrain.getPose())).toPose2d().getTranslation(), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)) // position, heading(direction of travel), holonomic rotation
                new PathPoint(endPoint.plus(new Translation2d(0.127, 0)), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180)), // position, heading(direction of travel), holonomic rotation
                new PathPoint(endPoint, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180)) // position, heading(direction of travel), holonomic rotation
        );
        field2d.getObject("gridDrivePath").setTrajectory(trajectory);
        field2d.getObject("endPose").setPose(trajectory.getEndState().poseMeters);
        PathPlannerServer.sendActivePath(trajectory.getStates());

        addCommands(drivetrain.getPathCommand(trajectory));
        addRequirements(drivetrain);
    }

}