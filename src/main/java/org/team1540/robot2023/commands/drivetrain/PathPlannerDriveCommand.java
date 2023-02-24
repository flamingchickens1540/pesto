package org.team1540.robot2023.commands.drivetrain;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PathPlannerDriveCommand extends SequentialCommandGroup{
    public PathPlannerDriveCommand(Drivetrain drivetrain, String pathName) {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath(pathName, new PathConstraints(3, 2));
        trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, DriverStation.getAlliance());
        PathPlannerServer.sendActivePath(trajectory.getStates());
        Command ramseteCommand = drivetrain.getPathCommand(trajectory);
        addRequirements(drivetrain);
        addCommands(ramseteCommand);
    }

}
