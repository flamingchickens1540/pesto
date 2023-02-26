package org.team1540.robot2023.commands.arm;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ManualArm extends ParallelCommandGroup {
    public ManualArm(Arm arm, CommandXboxController controller) {
        addCommands(
                new ManualRotation(arm, controller),
                new ManualExtension(arm, controller)
        );
    }
}
