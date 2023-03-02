package org.team1540.robot2023.commands.drivetrain;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2023.commands.auto.AutoDrive;
import org.team1540.robot2023.utils.GridScoreData;

import static org.team1540.robot2023.commands.auto.AutoDrive.getClosestTag;

class GridDriveCommand extends SequentialCommandGroup {

    public GridDriveCommand(Drivetrain drivetrain, GridScoreData position) {
        this(drivetrain, getClosestTag(drivetrain), position);
    }
    public GridDriveCommand(Drivetrain drivetrain, int tag, GridScoreData position) {
        Translation2d endPoint = AutoDrive.getGridDrivePose(drivetrain, position);
        addCommands(
                AutoDrive.driveToPoints(drivetrain,
                    new PathPoint(endPoint.plus(new Translation2d(0.127, 0)), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180)), // position, heading(direction of travel), holonomic rotation
                    new PathPoint(endPoint, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180)) // position, heading(direction of travel), holonomic rotation)
        ));
    }

}