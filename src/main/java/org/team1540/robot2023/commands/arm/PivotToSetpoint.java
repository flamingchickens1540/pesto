package org.team1540.robot2023.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PivotToSetpoint extends SequentialCommandGroup {
    public PivotToSetpoint(Arm arm, Rotation2d setpoint) {
        addCommands(
                new RetractExtension(arm),
                new PivotPID(arm, setpoint)
        );
        addRequirements(arm);
    }
}
