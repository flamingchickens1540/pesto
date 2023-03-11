package org.team1540.robot2023.utils;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.commands.drivetrain.PathPlannerDriveCommand;

import java.util.LinkedList;
import java.util.List;

public abstract class AutoCommand extends SequentialCommandGroup {
    private Trajectory fullTrajectory;
    private String name;

    public Command getPathPlannerDriveCommand(Drivetrain drivetrain, String pathname) {
        this.name = pathname;
        PathPlannerTrajectory trajectory = PathPlanner.loadPath(pathname, 1.5, 2);
        this.fullTrajectory = trajectory;
        return new ProxyCommand(() -> new PathPlannerDriveCommand(drivetrain, trajectory));
    }
    public List<Command> getPathPlannerDriveCommandGroup(Drivetrain drivetrain, String pathname) {
        this.name = pathname;
        List<PathPlannerTrajectory> trajectories = PathPlanner.loadPathGroup(pathname, 2, 1);
        LinkedList<Command> commands = new LinkedList<>();
        this.fullTrajectory = new Trajectory();
        for (PathPlannerTrajectory trajectory : trajectories) {
            this.fullTrajectory = this.fullTrajectory.concatenate(trajectory);
            commands.addLast(new ProxyCommand(() -> new PathPlannerDriveCommand(drivetrain, trajectory)));
        }
        return commands;
    }

    public void setName(String name) {
        this.name = name;
        this.fullTrajectory = new Trajectory();
    }
    public Trajectory getFullTrajectory() {
        return fullTrajectory;
    }
    public String getName() { return name;}
}
