package org.team1540.robot2023.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2023.commands.arm.*;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.commands.drivetrain.ProxiedGridDriveCommand;
import org.team1540.robot2023.commands.grabber.GrabberOuttakeCommand;
import org.team1540.robot2023.commands.grabber.WheeledGrabber;
import org.team1540.robot2023.utils.ArmState;
import org.team1540.robot2023.utils.PolePosition;

public class GridDriveAndPivotCommand extends SequentialCommandGroup{



    public GridDriveAndPivotCommand(Drivetrain drivetrain, PolePosition position, Arm arm, ArmState setpoint){
        addCommands(
            new ProxiedGridDriveCommand(drivetrain, position),
            new RetractAndPivotCommand(arm, setpoint.getRotation2d()),
            new ExtensionCommand(arm, setpoint.getExtension())
        );

    }

    public GridDriveAndPivotCommand(Drivetrain drivetrain, PolePosition position, Arm arm, WheeledGrabber grabber, ArmState setpoint){
        addCommands(
                new ProxiedGridDriveCommand(drivetrain, position),
                new RetractAndPivotCommand(arm, setpoint.getRotation2d()),
                new ExtensionCommand(arm, setpoint.getExtension()),
                new GrabberOuttakeCommand(grabber).withTimeout(0.5),
                new ResetArmPositionCommand(arm)
        );

    }

    public GridDriveAndPivotCommand(Drivetrain drivetrain, PolePosition position, Arm arm, WheeledGrabber grabber, ArmState setpoint, ArmState endpoint){
        addCommands(
                new ProxiedGridDriveCommand(drivetrain, position),
                new RetractAndPivotCommand(arm, setpoint.getRotation2d()),
                new ExtensionCommand(arm, setpoint.getExtension()),
                new PivotCommand(arm, endpoint.getRotation2d()),
                new GrabberOuttakeCommand(grabber).withTimeout(0.5),
                new ResetArmPositionCommand(arm)
        );

    }
}
