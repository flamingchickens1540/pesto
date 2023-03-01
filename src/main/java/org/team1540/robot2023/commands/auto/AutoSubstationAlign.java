package org.team1540.robot2023.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team1540.robot2023.Constants;
import org.team1540.robot2023.commands.arm.Arm;
import org.team1540.robot2023.commands.arm.ExtensionCommand;
import org.team1540.robot2023.commands.arm.PivotCommand;
import org.team1540.robot2023.commands.arm.ResetArmPositionCommand;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.commands.grabber.GrabberIntakeCommand;
import org.team1540.robot2023.commands.grabber.WheeledGrabber;

import static org.team1540.robot2023.Globals.aprilTagLayout;

public class AutoSubstationAlign extends SequentialCommandGroup {
    public AutoSubstationAlign(Drivetrain drivetrain, Arm arm, WheeledGrabber intake, CommandXboxController controller, double offset) {
        aprilTagLayout.setOrigin(DriverStation.getAlliance() == DriverStation.Alliance.Red ? AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide : AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
        Translation2d endPoint = aprilTagLayout.getTagPose(5).orElseThrow().toPose2d().getTranslation().plus(new Translation2d(-Constants.Auto.hpOffsetX, offset));

        addCommands(
            Commands.parallel(
                new GrabberIntakeCommand(intake),
                Commands.sequence(
                        Commands.parallel(
                    new ProxyCommand(() ->{
                        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
                            new PathConstraints(5, 3),
                            new PathPoint(drivetrain.getPose().getTranslation(), Rotation2d.fromDegrees(0), drivetrain.getPose().getRotation()), // position, heading(direction of travel), holonomic rotation
                            new PathPoint(endPoint.plus(new Translation2d(-0.381, 0)), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)) // position, heading(direction of travel), holonomic rotation
                        );
                        return drivetrain.getPathCommand(trajectory);
                    }),
                    new PivotCommand(arm, Constants.Auto.armHumanPlayer.getRotation2d())
                        ),
                    new ExtensionCommand(arm, Constants.Auto.armHumanPlayer.getExtension()),
                    new ProxyCommand(() ->{
                        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
                                new PathConstraints(5, 3),
                                new PathPoint(drivetrain.getPose().getTranslation(), Rotation2d.fromDegrees(0), drivetrain.getPose().getRotation()), // position, heading(direction of travel), holonomic rotation
                                new PathPoint(endPoint, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)) // position, heading(direction of travel), holonomic rotation
                        );
                        return drivetrain.getPathCommand(trajectory);
                    }),
                    new WaitUntilCommand(intake::hasGamePiece),
//                    new WaitUntilCommand(() -> controller.getLeftTriggerAxis() > 0.95),
                    new ProxyCommand(() ->{
                        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
                                new PathConstraints(5, 3),
                                new PathPoint(drivetrain.getPose().getTranslation(), Rotation2d.fromDegrees(0), drivetrain.getPose().getRotation()), // position, heading(direction of travel), holonomic rotation
                                new PathPoint(endPoint.plus(new Translation2d(-0.381, 0)), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)) // position, heading(direction of travel), holonomic rotation
                        );
                        return drivetrain.getPathCommand(trajectory);
                    }),
                    new ResetArmPositionCommand(arm)
                )
            )
        );
//                new PrintCommand("Done")
//        addRequirements(drivetrain,arm);
    }
}
