package org.team1540.robot2023.commands.drivetrain;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PathPlannerDriveCommand extends SequentialCommandGroup {
    public PathPlannerDriveCommand(Drivetrain drivetrain) {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("ChargeStation", new PathConstraints(3, 2));
        PathPlannerServer.sendActivePath(trajectory.getStates());
        drivetrain.setFieldPath(trajectory);
        Command ramseteCommand = drivetrain.getPathCommand(trajectory);
        addRequirements(drivetrain);
        addCommands(ramseteCommand);
    }
}
