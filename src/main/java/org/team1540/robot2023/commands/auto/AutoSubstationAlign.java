package org.team1540.robot2023.commands.auto;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team1540.robot2023.Constants;
import org.team1540.robot2023.commands.arm.Arm;
import org.team1540.robot2023.commands.arm.ResetArmPositionCommand;
import org.team1540.robot2023.commands.arm.SetArmPosition;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.commands.grabber.GrabberIntakeCommand;
import org.team1540.robot2023.commands.grabber.WheeledGrabber;
import org.team1540.robot2023.utils.ArmState;

import static org.team1540.robot2023.Globals.aprilTagLayout;

public class AutoSubstationAlign extends SequentialCommandGroup {
    private AutoSubstationAlign(Drivetrain drivetrain, Arm arm, WheeledGrabber intake, CommandXboxController controller, double offset) {
        boolean didSucceed = drivetrain.updateWithScoringApriltags();
        if (!didSucceed) {
            addCommands(new PrintCommand("NOT SUBSTATION ALIGNING - NO TAG"));
            return;
        }
        aprilTagLayout.setOrigin(DriverStation.getAlliance() == DriverStation.Alliance.Red ? AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide : AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
        int substationTagID = DriverStation.getAlliance() == DriverStation.Alliance.Red ? 5 : 4;
        Translation2d endPoint = aprilTagLayout.getTagPose(substationTagID).orElseThrow().toPose2d().getTranslation().plus(new Translation2d(-Constants.Auto.hpOffsetX, offset));

        addCommands(
                Commands.race(
                        new RepeatCommand(new GrabberIntakeCommand(intake)),
                        Commands.sequence(
                                Commands.parallel(
                                        AutoDrive.driveToPoints(drivetrain, new PathPoint(endPoint.plus(new Translation2d(-Units.inchesToMeters(20), 0)), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0))).andThen(new PrintCommand("Done driving")),
                                        new SetArmPosition(arm, ArmState.fromRotationExtension(Constants.Auto.armHumanPlayer.getRotation2d(),arm.getArmState().getExtension()))
                                ),
                                new SetArmPosition(arm, Constants.Auto.armHumanPlayer).andThen(new PrintCommand("Done extending")),
                                AutoDrive.driveToPoints(drivetrain, 0.5, 1, new PathPoint(endPoint, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0))),
                                new WaitUntilCommand(() -> intake.hasGamePiece() || controller.getLeftTriggerAxis() > 0.95),
                                //                  new WaitUntilCommand(() -> controller.getLeftTriggerAxis() > 0.95),
                                new SetArmPosition(arm, Constants.Auto.armHumanPlayerRetreat).withTimeout(5),
//                                AutoDrive.driveToPoints(drivetrain,new PathPoint(endPoint.plus(new Translation2d(-0.381, 0)), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0))),
                        new ResetArmPositionCommand(arm)
                        )
                )
        );
    }

    public static Command get(Drivetrain drivetrain, Arm arm, WheeledGrabber intake, CommandXboxController controller, double offset) {
        return new ProxyCommand(() -> new AutoSubstationAlign(drivetrain, arm, intake, controller, offset)).withName("AutoSubstationAlignProxier");
    }
//                new PrintCommand("Done")
//        addRequirements(drivetrain,arm);
}
