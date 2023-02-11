package org.team1540.robot2023.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProxyCommand;

import java.util.function.Supplier;

// This was the best way I found to force GridDriveCommand to be constructed at schedule time rather than command construction time
public class ProxiedGridDriveCommand extends ProxyCommand {

    /**
     * Constructs a ProxiedGridDriveCommand. It will construct a new GridDriveCommand when it is run?
     * @param drivetrain
     * @param tagID
     */
    public ProxiedGridDriveCommand(Drivetrain drivetrain, int tagID) {
        super(() -> new GridDriveCommand(drivetrain, tagID));
    }
}
