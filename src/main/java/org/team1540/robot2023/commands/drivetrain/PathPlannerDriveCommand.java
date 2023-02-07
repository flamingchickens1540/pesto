package org.team1540.robot2023.commands.drivetrain;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.HashMap;

public class PathPlannerDriveCommand extends SequentialCommandGroup {
    public PathPlannerDriveCommand(Drivetrain drivetrain) {
        addRequirements(drivetrain);
        drivetrain.zeroGyroscope();
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("EpicSwervyThing", new PathConstraints(3, 2));
        Command ramseteCommand = drivetrain.getResettingPathCommand(trajectory);
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("testevent", new SequentialCommandGroup(new PrintCommand("starting"), new WaitCommand(3), new PrintCommand("ending")));

        FollowPathWithEvents command = new FollowPathWithEvents(
                ramseteCommand,
                trajectory.getMarkers(),
                eventMap
        );
        addCommands(command);
    }
}
