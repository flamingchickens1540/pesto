package org.team1540.robot2023;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoCommandData {
    public final Command command;
    public final String name;
    public final Trajectory trajectory;
    public final boolean resetting;
    public final Pose2d initialPose;

    public AutoCommandData(Command command, String name, Trajectory trajectory, boolean resetting) {
        this.command = command;
        this.name = name;
        this.trajectory = trajectory;
        this.resetting = resetting;
        this.initialPose = null;
    }
    public AutoCommandData(Command command, String name, Trajectory trajectory, Pose2d initialPose, boolean resetting) {
        this.command = command;
        this.name = name;
        this.trajectory = trajectory;
        this.resetting = resetting;
        this.initialPose = initialPose;
    }
}
