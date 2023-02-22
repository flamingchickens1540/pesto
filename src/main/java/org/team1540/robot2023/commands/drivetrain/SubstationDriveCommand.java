package org.team1540.robot2023.commands.drivetrain;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2023.Constants;

import static org.team1540.robot2023.Globals.aprilTagLayout;

class SubstationDriveCommand extends SequentialCommandGroup {
    public SubstationDriveCommand(Drivetrain drivetrain, double yOffset) {
        Translation2d endPoint = aprilTagLayout.getTagPose(4).orElseThrow().toPose2d().getTranslation().plus(new Translation2d(Constants.Auto.hpOffsetX, yOffset));

        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
                new PathConstraints(5, 3),
                new PathPoint(drivetrain.getPose().getTranslation(), Rotation2d.fromDegrees(0), drivetrain.getPose().getRotation()), // position, heading(direction of travel), holonomic rotation
//                new PathPoint(layout.getTagPose(tag).orElse(new Pose3d(drivetrain.getPose())).toPose2d().getTranslation(), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)) // position, heading(direction of travel), holonomic rotation
                new PathPoint(endPoint.plus(new Translation2d(0.127, 0)), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180)), // position, heading(direction of travel), holonomic rotation
                new PathPoint(endPoint, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180)) // position, heading(direction of travel), holonomic rotation
        );
        PathPlannerServer.sendActivePath(trajectory.getStates());

        addCommands(drivetrain.getPathCommand(trajectory));
        addRequirements(drivetrain);
    }

}