package org.team1540.robot2023;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.team1540.lib.util.TrajectoryTransformer;
import org.team1540.robot2023.utils.AutoCommand;

import static org.team1540.robot2023.Globals.field2d;

public class AutoManager {

    private static AutoManager instance;
    private final SendableChooser<AutoCommandData> chooser = new SendableChooser<>();

    public static AutoManager getInstance() {
        if (instance == null) {
            instance = new AutoManager();
        }
        return instance;
    }
    private AutoManager() {
        SmartDashboard.putData("autoChooser", chooser);
    }

    public void addAuto(AutoCommand command) {
        addAuto(command.getName(), command, command.getFullTrajectory());
    }
    public void addAuto(String name, AutoCommand command) {
        addAuto(name, command, command.getFullTrajectory());
    }
    public void addAuto(String name, Command command) {
        chooser.addOption(name, new AutoCommandData(command, name, null));
    }
    public void addAuto(String name, Command command, Trajectory trajectory) {
        if (trajectory != null) {
            field2d.getObject("trajectory/" + name).setTrajectory(trajectory);
        }
        chooser.addOption(name, new AutoCommandData(command, name, trajectory));
    }
    public void addDefaultAuto(String name, Command command, Trajectory trajectory) {
        chooser.setDefaultOption(name, new AutoCommandData(command, name, trajectory));
    }

    public void updateSelected() {
        AutoCommandData selected = chooser.getSelected();
        if (selected != null) {
            if (selected.trajectory != null) {
                field2d.getObject("trajectory").setTrajectory(TrajectoryTransformer.transformTrajectoryForAlliance(selected.trajectory, DriverStation.getAlliance()));
            } else {
                field2d.getObject("trajectory").setPoses();
            }
        }
    }

    public Command getSelected() {
        return chooser.getSelected().command;
    }
}
