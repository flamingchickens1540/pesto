package org.team1540.robot2023.commands.auto;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.utils.GridScoreData;

import java.util.function.BooleanSupplier;

public class AutoGridAlign extends SequentialCommandGroup {

    public AutoGridAlign(Drivetrain drivetrain, GridScoreData positions, boolean shouldAlign){
        BooleanSupplier shouldRun;
        if (shouldAlign) {
            shouldRun = drivetrain::updateWithScoringApriltags;
        } else {
            shouldRun = () -> true;
        }
        addCommands(
                new ProxyCommand(() -> {
                    Translation2d endPoint = AutoDrive.getGridDrivePose(drivetrain, positions);
                    return AutoDrive.driveToPoints(
                            drivetrain,
                            new PathPoint(endPoint.plus(new Translation2d(Units.inchesToMeters(5),0)), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180)),
//                                                new PathPoint(endPoint, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180)).withPrevControlLength(Units.inchesToMeters(8))
                            new PathPoint(endPoint, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180))
                    );
                }
                ).unless(()->!shouldAlign)
        );
    }
}
