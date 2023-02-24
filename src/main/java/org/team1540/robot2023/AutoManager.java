package org.team1540.robot2023;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.team1540.robot2023.utils.AutoCommand;

import static org.team1540.robot2023.Globals.field2d;

public class AutoManager {
    private static AutoManager instance;
    private final SendableChooser<AutoCommand> chooser = new SendableChooser<>();

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
        chooser.addOption(command.getName(), command);
    }

    public void updateSelected() {
        AutoCommand selected = chooser.getSelected();
        if (selected != null) {
            field2d.getObject("trajectory").setTrajectory(PathPlannerTrajectory.transformTrajectoryForAlliance(selected.getTrajectory(), DriverStation.getAlliance()));
        }
    }

    public Command getSelected() {
        return chooser.getSelected();
    }
}
