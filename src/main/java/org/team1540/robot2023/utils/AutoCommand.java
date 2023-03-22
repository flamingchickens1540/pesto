package org.team1540.robot2023.utils;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.commands.drivetrain.PathPlannerDriveCommand;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

public abstract class AutoCommand extends SequentialCommandGroup {
    private Trajectory fullTrajectory;
    private String name;

    public Command getPathPlannerDriveCommand(Drivetrain drivetrain, String pathname) {
        return getPathPlannerDriveCommand(drivetrain, pathname, new PathConstraints(4, 2), false);
    }
    public Command getPathPlannerDriveCommand(Drivetrain drivetrain, String pathname, PathConstraints constraints, boolean shouldReset) {
        this.name = pathname;
        PathPlannerTrajectory trajectory = PathPlanner.loadPath(pathname, constraints);
        this.fullTrajectory = trajectory;
        Command command = new ProxyCommand(() -> new PathPlannerDriveCommand(drivetrain, trajectory));
        if (shouldReset) {
            return new InstantCommand(() -> drivetrain.resetToPath(trajectory)).andThen(command);
        } else {
            return command;
        }
    }

    public List<Command> getPathPlannerDriveCommandGroup(Drivetrain drivetrain, String pathname) {
        return getPathPlannerDriveCommandGroup(drivetrain, pathname, new PathConstraints(4, 2), false);
    }

    public List<Command> getPathPlannerDriveCommandGroup(Drivetrain drivetrain, String pathname, PathConstraints constraints, boolean shouldReset) {
        return getPathPlannerDriveCommandGroup(drivetrain, pathname, new PathConstraints[]{constraints}, false);
    }


    public List<Command> getPathPlannerDriveCommandGroup(Drivetrain drivetrain, String pathname, PathConstraints[] constraints, boolean shouldReset) {
        this.name = pathname;
        List<PathPlannerTrajectory> trajectories = PathPlanner.loadPathGroup(pathname, constraints[0], Arrays.copyOfRange(constraints, 1, constraints.length));
        LinkedList<Command> commands = new LinkedList<>();
        this.fullTrajectory = new Trajectory();
        for (PathPlannerTrajectory trajectory : trajectories) {
            this.fullTrajectory = this.fullTrajectory.concatenate(trajectory);
            commands.addLast(new ProxyCommand(() -> new PathPlannerDriveCommand(drivetrain, trajectory)));
        }
        if (shouldReset) {
            commands.set(0,new InstantCommand(() -> drivetrain.resetToPath(trajectories.get(0))).andThen(commands.getFirst()));
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
