package org.team1540.robot2023.utils;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.commands.drivetrain.PathPlannerDriveCommand;

public abstract class AutoCommand extends SequentialCommandGroup {
    private PathPlannerTrajectory trajectory;
    private String name;

    public CommandBase getPathPlannerDriveCommand(Drivetrain drivetrain, String pathname) {
        this.name = pathname;
        this.trajectory = PathPlanner.loadPath(pathname, 1, 1);
        return new ProxyCommand(() -> new PathPlannerDriveCommand(drivetrain, pathname));
    }
    public PathPlannerTrajectory getTrajectory() {
        return trajectory;
    }
    public String getName() { return name;}
}
