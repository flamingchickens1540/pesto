package org.team1540.robot2023.commands.drivetrain;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.HashMap;

public class PathPlannerDriveCommand extends SequentialCommandGroup {
    public PathPlannerDriveCommand(Drivetrain drivetrain) {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("EpicSwervyThing", new PathConstraints(3, 2));
        PathPlannerServer.sendActivePath(trajectory.getStates());
        Command ramseteCommand = drivetrain.getResettingPathCommand(trajectory);
        addRequirements(drivetrain);
        addCommands(ramseteCommand);
    }
}
