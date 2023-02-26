package org.team1540.robot2023.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import org.team1540.robot2023.commands.arm.Arm;
import org.team1540.robot2023.commands.arm.PivotCommand;
import org.team1540.robot2023.commands.arm.RetractExtension;
import org.team1540.robot2023.commands.arm.SetArmPosition;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.commands.drivetrain.ProxiedGridDriveCommand;
import org.team1540.robot2023.commands.grabber.GrabberIntakeCommand;
import org.team1540.robot2023.commands.grabber.GrabberOuttakeCommand;
import org.team1540.robot2023.commands.grabber.WheeledGrabber;
import org.team1540.robot2023.utils.ArmState;
import org.team1540.robot2023.utils.PolePosition;

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
                    Commands.parallel(
                            alignmentCommand,
                            Commands.sequence(
                                new PrintCommand("Starting retract"),
                                new RetractExtension(arm),
                                new PrintCommand("Starting adjust position"),
                                new SetArmPosition(arm, approachSetpoint)
                            )
                            
                    )
            ),
            new PrintCommand("Starting adjust position v2"),
            new SetArmPosition(arm, setpoint),
            new PrintCommand("Starting retreat"),
            Commands.parallel(
                new GrabberOuttakeCommand(intake),
                Commands.sequence(
                    new PrintCommand("Starting retreat"),
                    new WaitCommand(0.25),
                    Commands.parallel(
                        new PivotCommand(arm, approachSetpoint.getRotation2d()),
                        new RetractExtension(arm)
                    )
                )
            )
            
        );
    }
}
