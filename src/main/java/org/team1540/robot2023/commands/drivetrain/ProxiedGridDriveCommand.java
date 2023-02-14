package org.team1540.robot2023.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import org.team1540.robot2023.utils.PolePosition;

import java.util.function.Supplier;

// This was the best way I found to force GridDriveCommand to be constructed at schedule time rather than command construction time
public class ProxiedGridDriveCommand extends ProxyCommand {

    /**
     * Constructs a ProxiedGridDriveCommand. It will construct a new GridDriveCommand when it is run and use the given tag
     * @param drivetrain the drivetrain subsystem
     * @param tagID the ID of the tag to drive to
     * @param position Which side of the grid to move to
     */
    public ProxiedGridDriveCommand(Drivetrain drivetrain, int tagID, PolePosition position) {
        super(() -> new GridDriveCommand(drivetrain, tagID, position));
    }
    /**
     * Constructs a ProxiedGridDriveCommand. It will construct a new GridDriveCommand when it is run and use the closest tag
     * @param drivetrain the drivetrain subsystem
     * @param position Which side of the grid to move to
     */
    public ProxiedGridDriveCommand(Drivetrain drivetrain, PolePosition position) {
        super(() -> new GridDriveCommand(drivetrain, position));
    }
}
