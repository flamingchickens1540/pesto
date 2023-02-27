package org.team1540.robot2023.commands.drivetrain;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2023.Constants;
import org.team1540.robot2023.utils.PolePosition;

import static org.team1540.robot2023.Globals.aprilTagLayout;
import static org.team1540.robot2023.Globals.field2d;

class GridDriveCommand extends SequentialCommandGroup {
    private static int getClosestTag(Drivetrain drivetrain) {
        aprilTagLayout.setOrigin(DriverStation.getAlliance() == DriverStation.Alliance.Red ? AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide : AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
        Translation2d currentPose = drivetrain.getPose().getTranslation();
        double mindist = Double.MAX_VALUE;
        int closestTag = -1;
        try {
            for (AprilTag tag : aprilTagLayout.getTags()) {
                Pose2d pose = aprilTagLayout.getTagPose(tag.ID).orElseThrow().toPose2d();
                double distance = pose.getTranslation().getDistance(currentPose);
                field2d.getObject("apriltag/" + tag.ID).setPose(pose);
                if (distance < mindist || closestTag == -1) {
                    mindist = distance;

                    closestTag = tag.ID;
                }
            }
        }  catch (Exception e) {
            DriverStation.reportError("COULDN'T FIND APRIL TAG FOR SOME REASON THIS IS NOT SUPPOSED TO HAPPEN", false);
        }
        SmartDashboard.putNumber("drivetrain/tag", closestTag);
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
//        PathPlannerTrajectory flippedTrajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, DriverStation.getAlliance());
        addCommands(drivetrain.getPathCommand(trajectory));
        addRequirements(drivetrain);
    }

}