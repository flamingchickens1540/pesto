package org.team1540.robot2023.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2023.commands.arm.Arm;
import org.team1540.robot2023.commands.arm.SetArmPosition;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.commands.drivetrain.ProxiedGridDriveCommand;
import org.team1540.robot2023.commands.grabber.GrabberIntakeCommand;
import org.team1540.robot2023.commands.grabber.GrabberOuttakeCommand;
import org.team1540.robot2023.commands.grabber.WheeledGrabber;
import org.team1540.robot2023.utils.ArmState;
import org.team1540.robot2023.utils.PolePosition;

public class AutoGridScore extends SequentialCommandGroup {


    public AutoGridScore(Drivetrain drivetrain, PolePosition position, Arm arm, ArmState setpoint, WheeledGrabber intake){
        this(drivetrain, position, arm, setpoint, intake, true);
    }
    public AutoGridScore(Drivetrain drivetrain, PolePosition position, Arm arm, ArmState setpoint, WheeledGrabber intake, boolean shouldAlign){
        Command alignmentCommand = shouldAlign ? new ProxiedGridDriveCommand(drivetrain, position) : new InstantCommand();
        addCommands(
            Commands.race(
                    new GrabberIntakeCommand(intake),
                    Commands.sequence(
                            alignmentCommand,
                            new SetArmPosition(arm, setpoint)
                    )
            ),
            new GrabberOuttakeCommand(intake)
        );
    }
}
