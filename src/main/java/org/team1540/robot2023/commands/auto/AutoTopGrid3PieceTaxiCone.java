package org.team1540.robot2023.commands.auto;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import org.team1540.robot2023.Constants;
import org.team1540.robot2023.commands.arm.*;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.commands.grabber.GrabberAggressiveCommand;
import org.team1540.robot2023.commands.grabber.GrabberIntakeCommand;
import org.team1540.robot2023.commands.grabber.GrabberOuttakeCommand;
import org.team1540.robot2023.commands.grabber.WheeledGrabber;
import org.team1540.robot2023.utils.ArmState;
import org.team1540.robot2023.utils.AutoCommand;
import org.team1540.robot2023.utils.PolePosition;

import java.util.List;

public class AutoTopGrid3PieceTaxiCone extends AutoCommand {
    public AutoTopGrid3PieceTaxiCone(Drivetrain drivetrain, Arm arm, WheeledGrabber intake) {
        long startTime = System.currentTimeMillis();
        List<Command> pathCommands = getPathPlannerDriveCommandGroup(drivetrain, "TopGrid3PieceTaxiCone", new PathConstraints(5, 3.3), true);
        addCommands(
//                new AutoCone(drivetrain, arm, Constants.Auto.highCone.withPolePosition(PolePosition.LEFT), intake, null, false),
                Commands.deadline(
                        new SetArmPosition(arm, Constants.Auto.highCone.approach),
                        new GrabberAggressiveCommand(intake)
                ),
                new SetArmPosition(arm, AutoHybrid.catchNull(Constants.Auto.highCone.score)),
                new GrabberOuttakeCommand(intake, 1).withTimeout(0.1),
                new SetArmPosition(arm, AutoHybrid.catchNull(Constants.Auto.highCone.retreat)).unless(() -> Constants.Auto.highCone.retreat == null),
                Commands.parallel(
                        new GrabberIntakeCommand(intake),
                        Commands.sequence(
                                Commands.parallel(
                                        Commands.sequence(
                                                new WaitCommand(0.1),
                                                pathCommands.get(0)

                                        ),
                                        Commands.sequence(
//                                                new WaitCommand(0.1),
                                                new ResetArmPositionCommand(arm),
//                                                new ResetArmPositionCommand(arm, true),
                                                new PivotCommand(arm, Constants.Auto.armDownBackwards)
                                        )
                                ),
                                Commands.parallel(
                                        new PivotCommand(arm, Constants.Auto.highCube.approach),
                                        pathCommands.get(1)
                                )
                        )
                ),
                new InstantCommand(drivetrain::updateWithScoringApriltags),//
//                new WaitCommand(0.1),
//                new SetArmPosition(arm, Constants.Auto.highCube.approach),
//                new GrabberOuttakeCommand(intake, 1).withTimeout(0.1),
                Commands.deadline(
                        new SetArmPosition(arm, Constants.Auto.highCube.approach),
                        Commands.sequence(
                                new ProxyCommand(() -> new WaitCommand((arm.timeToState(Constants.Auto.highCube.approach)-150)/1000)),
                                new GrabberOuttakeCommand(intake, 0.5)
                        )
                ),
                //        new AutoCube(drivetrain, arm, Constants.Auto.midCube.withPolePosition(PolePosition.CENTER), intake, false),
                Commands.parallel(
                        pathCommands.get(2),
                        Commands.sequence(
                                new ResetArmPositionCommand(arm),
                                new SetArmPosition(arm, Constants.Auto.reverseCube),
                                new GrabberIntakeCommand(intake)
                        )
                ),
                Commands.parallel(
                        new SetArmPosition(arm, Constants.Auto.midCube.approach),
                        pathCommands.get(3)
                ),
                new InstantCommand(drivetrain::updateWithScoringApriltags),
                new GrabberOuttakeCommand(intake,0.5),
                new InstantCommand(() -> System.out.println("Time taken in ms: " + (System.currentTimeMillis() - startTime)))
//                new AutoHybrid(drivetrain, arm, Constants.Auto.hybridNode.withPolePosition(PolePosition.CENTER), intake, null, true)

        );
    }


}