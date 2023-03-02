package org.team1540.robot2023.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;

public class Rotate extends SequentialCommandGroup {
    Drivetrain drivetrain;
    Command command;
    public Rotate(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addCommands(
                new ProxyCommand(() -> {
                    PathPlannerTrajectory trajectory = PathPlanner.generatePath(
                            new PathConstraints(5, 3),
                            new PathPoint(drivetrain.getPose().getTranslation(), Rotation2d.fromDegrees(0), drivetrain.getPose().getRotation()), // position, heading(direction of travel), holonomic rotation
                            new PathPoint(drivetrain.getPose().getTranslation(), Rotation2d.fromDegrees(0), drivetrain.getPose().getRotation().getDegrees()+90) // position, heading(direction of travel), holonomic rotation
                    );
                    return drivetrain.getPathCommand(trajectory);
                })
        );
//        addRequirements(drivetrain);
    }

//    @Override
//    public void initialize() {
//
//            command.initialize();
//    }
//
//    @Override
//    public void execute() {
//        command.execute();
//    }
//
//    @Override
//    public void end(boolean ignored) {
//        command.end(ignored);
//    }
}
