package org.team1540.robot2023;


import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoCommandData {
    public final Command command;
    public final String name;
    public final Trajectory trajectory;

    public AutoCommandData(Command command, String name, Trajectory trajectory) {
        this.command = command;
        this.name = name;
        this.trajectory = trajectory;
    }
}
