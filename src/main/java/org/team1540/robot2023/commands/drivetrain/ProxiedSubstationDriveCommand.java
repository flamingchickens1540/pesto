package org.team1540.robot2023.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.ProxyCommand;

// This was the best way I found to force GridDriveCommand to be constructed at schedule time rather than command construction time
public class ProxiedSubstationDriveCommand extends ProxyCommand {

    public ProxiedSubstationDriveCommand(Drivetrain drivetrain, double position) {
        super(() -> new SubstationDriveCommand(drivetrain, position));
    }
}
