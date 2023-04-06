package org.team1540.robot2023.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import org.team1540.robot2023.Constants;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.utils.GridScoreData;

import java.util.LinkedList;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.function.Supplier;

import static org.team1540.robot2023.Globals.aprilTagLayout;
import static org.team1540.robot2023.Globals.field2d;

public class AutoDrive {
    public static PIDController alignmentTranslationPID = new PIDController(5,0,0);
    public static PIDController alignmentRotationPID = new PIDController(2,0,0);

    public static void postPIDs() {
        SmartDashboard.putData("align/rotationPID", alignmentRotationPID);
        SmartDashboard.putData("align/translationPID", alignmentTranslationPID);
    }
    public static int getClosestTag(Drivetrain drivetrain) {
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
        }  catch (NoSuchElementException e) {
            DriverStation.reportError("COULDN'T FIND APRIL TAG FOR SOME REASON THIS IS NOT SUPPOSED TO HAPPEN", false);
        }
        SmartDashboard.putNumber("drivetrain/tag", closestTag);
        return closestTag;
    }

    public static Pose2d getClosestTagPose(Drivetrain drivetrain) {
        return aprilTagLayout.getTagPose(getClosestTag(drivetrain)).orElseThrow().toPose2d();
    }
    public static Translation2d getGridDrivePose(Drivetrain drivetrain, GridScoreData data) {
        return getClosestTagPose(drivetrain).getTranslation().plus(new Translation2d(Constants.Auto.gridBackoffOffsetMeters+data.additionalBackoff, data.polePosition.offset));
    }

    public static Command driveToPoints(Drivetrain drivetrain, PathPoint... points) {
        return driveToPoints(drivetrain, 5, 3, points);
    }

    public static Command driveToPoints(Drivetrain drivetrain, double maxVelocity, double maxAcceleration, PathPoint... points) {
        return driveToPoints(drivetrain, maxVelocity, maxAcceleration, () -> List.of(points));
    }

    public static Command driveToPoints(Drivetrain drivetrain, double maxVelocity, double maxAcceleration, Supplier<List<PathPoint>> points) {
        return new ProxyCommand(() -> {
            List<PathPoint> pointList = new LinkedList<>();
            long time = System.currentTimeMillis();
            pointList.add(new PathPoint(drivetrain.getPose().getTranslation(), Rotation2d.fromDegrees(0), drivetrain.getPose().getRotation()));
            pointList.addAll(points.get());
            PathPlannerTrajectory trajectory = PathPlanner.generatePath(
                    new PathConstraints(5, 3),
                    pointList
            );
            System.out.println("Time to generate path in ms: " + (System.currentTimeMillis() - time));
            field2d.getObject("gridDrivePath").setTrajectory(trajectory);
            field2d.getObject("endPose").setPose(trajectory.getEndState().poseMeters);
            PathPlannerServer.sendActivePath(trajectory.getStates());
            return drivetrain.getPathCommand(trajectory, alignmentTranslationPID, alignmentRotationPID);
        }).withName("AutoDriveToPoints");
    }


    public static Command smoothDriveToPoints(Drivetrain drivetrain, double maxVelocity, double maxAcceleration, double time, Supplier<List<PathPoint>> points) {
        return new ProxyCommand(() -> {
            List<PathPoint> pointList = new LinkedList<>();
            Translation2d position = drivetrain.getPose().getTranslation().plus(
                    new Translation2d(drivetrain.getChassisSpeeds().vxMetersPerSecond * time,
                            drivetrain.getChassisSpeeds().vyMetersPerSecond * time).rotateBy(drivetrain.getPose().getRotation())
            );

            pointList.add(new PathPoint(
                    position,
                    position.getAngle(),
                    Rotation2d.fromRadians(drivetrain.getPose().getRotation().getRadians() + drivetrain.getChassisSpeeds().omegaRadiansPerSecond*time),
                    Math.hypot(drivetrain.getChassisSpeeds().vxMetersPerSecond, drivetrain.getChassisSpeeds().vyMetersPerSecond)
                    )
            );
            pointList.addAll(points.get());
            PathPlannerTrajectory trajectory = PathPlanner.generatePath(
                    new PathConstraints(5, 3),
                    pointList
            );
            field2d.getObject("gridDrivePath").setTrajectory(trajectory);
            field2d.getObject("endPose").setPose(trajectory.getEndState().poseMeters);
            PathPlannerServer.sendActivePath(trajectory.getStates());
            return drivetrain.getPathCommand(trajectory, alignmentTranslationPID, alignmentRotationPID);
        }).withName("SmoothAutoDriveToPoints");
    }
}
