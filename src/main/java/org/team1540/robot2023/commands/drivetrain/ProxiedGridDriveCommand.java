package org.team1540.robot2023.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProxyCommand;

import java.util.function.Supplier;

public class ProxiedGridDriveCommand extends ProxyCommand {

    public ProxiedGridDriveCommand(Drivetrain drivetrain, int tagID) {
        this(() -> new GridDriveCommand(drivetrain, tagID));
    }

    /**
     * Creates a new ProxyCommand that schedules the supplied command when initialized, and ends when
     * it is no longer scheduled. Useful for lazily creating commands at runtime.
     *
     * @param supplier the command supplier
     */
    public ProxiedGridDriveCommand(Supplier<Command> supplier) {
        super(supplier);
    }
}
