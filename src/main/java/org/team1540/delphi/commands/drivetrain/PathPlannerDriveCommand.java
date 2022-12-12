package org.team1540.delphi.commands.drivetrain;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PathPlannerDriveCommand extends SequentialCommandGroup {
    public PathPlannerDriveCommand(Drivetrain drivetrain) {
        addRequirements(drivetrain);
        drivetrain.zeroGyroscope();
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("TestPath", new PathConstraints(3, 2));
        Command ramseteCommand = drivetrain.getResettingPathCommand(trajectory);


        addCommands(ramseteCommand);
    }
}
