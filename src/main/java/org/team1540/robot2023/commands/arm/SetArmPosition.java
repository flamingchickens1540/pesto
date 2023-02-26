package org.team1540.robot2023.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2023.utils.ArmState;

public class SetArmPosition extends SequentialCommandGroup {
    public SetArmPosition(Arm arm, ArmState setpoint) {
        addCommands(
                new RetractAndPivotCommand(arm,setpoint.getRotation2d()),
                new ExtensionCommand(arm,setpoint.getExtension())
        );
    }
}
