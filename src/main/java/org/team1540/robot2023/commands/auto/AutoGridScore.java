package org.team1540.robot2023.commands.auto;

import edu.wpi.first.wpilibj2.command.*;
import org.team1540.robot2023.commands.arm.*;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.commands.drivetrain.ProxiedGridDriveCommand;
import org.team1540.robot2023.commands.grabber.GrabberIntakeCommand;
import org.team1540.robot2023.commands.grabber.GrabberOuttakeCommand;
import org.team1540.robot2023.commands.grabber.WheeledGrabber;
import org.team1540.robot2023.utils.ArmState;
import org.team1540.robot2023.utils.PolePosition;

import java.util.Set;

public class AutoGridScore extends SequentialCommandGroup {

    public AutoGridScore(Drivetrain drivetrain, PolePosition position, Arm arm, ArmState setpoint, ArmState approachSetpoint, WheeledGrabber intake){
        this(drivetrain, position, arm, setpoint, approachSetpoint, intake, true);
    }
    public AutoGridScore(Drivetrain drivetrain, PolePosition position, Arm arm, ArmState setpoint, WheeledGrabber intake, boolean shouldAlign){
        this(drivetrain, position, arm, setpoint, setpoint, intake, shouldAlign);
    }
    public AutoGridScore(Drivetrain drivetrain, PolePosition position, Arm arm, ArmState setpoint, WheeledGrabber intake){
        this(drivetrain, position, arm, setpoint, setpoint, intake, true);
    }
    public AutoGridScore(Drivetrain drivetrain, PolePosition position, Arm arm, ArmState setpoint, ArmState approachSetpoint, WheeledGrabber intake, boolean shouldAlign){
        Command alignmentCommand = shouldAlign ? new ProxiedGridDriveCommand(drivetrain, position) : new InstantCommand();
        addCommands(
            Commands.race(
                    new GrabberIntakeCommand(intake),
                    Commands.sequence(
                            Commands.parallel(
                                    new PrintCommand("START MOVING"),
                                    alignmentCommand,
                                    new RetractAndPivotCommand(arm, approachSetpoint.getRotation2d())

                            ),
                            new SetArmPosition(arm, approachSetpoint)
                    )

            ),
            new PivotCommand(arm,setpoint.getRotation2d()),
            Commands.race(
                new GrabberOuttakeCommand(intake),
                Commands.sequence(
                    new WaitCommand(1),
                        new RetractAndPivotCommand(arm, approachSetpoint.getRotation2d())
                )
            ),
            new ResetArmPositionCommand(arm)
        );
    }
}
