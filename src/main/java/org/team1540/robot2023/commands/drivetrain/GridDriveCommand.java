package org.team1540.robot2023.commands.drivetrain;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team1540.robot2023.Constants;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import static org.team1540.robot2023.Globals.aprilTagLayout;
import static org.team1540.robot2023.utils.MathUtils.deadzone;

class GridDriveCommand extends SequentialCommandGroup {

    public GridDriveCommand(Drivetrain drivetrain, int tag) {

        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
                new PathConstraints(4, 3),
                new PathPoint(drivetrain.getPose().getTranslation(), Rotation2d.fromDegrees(0), drivetrain.getPose().getRotation()), // position, heading(direction of travel), holonomic rotation
//                new PathPoint(layout.getTagPose(tag).orElse(new Pose3d(drivetrain.getPose())).toPose2d().getTranslation(), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)) // position, heading(direction of travel), holonomic rotation
                new PathPoint(aprilTagLayout.getTagPose(tag).orElseThrow().toPose2d().getTranslation().plus(new Translation2d(1, 0)), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180)) // position, heading(direction of travel), holonomic rotation
        );
        PathPlannerServer.sendActivePath(trajectory.getStates());

        addCommands(drivetrain.getPathCommand(trajectory));
        addRequirements(drivetrain);
    }

}