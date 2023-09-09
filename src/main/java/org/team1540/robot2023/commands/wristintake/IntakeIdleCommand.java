package org.team1540.robot2023.commands.wristintake;

import org.team1540.robot2023.utils.GamePiece;

import java.util.function.Supplier;

import static org.team1540.robot2023.Constants.RollerIntakeConstants.*;

public class IntakeIdleCommand extends IntakeCommandBase {

    public IntakeIdleCommand(RollerIntake intake, Supplier<GamePiece> gamePieceSupplier) {
        super(intake, 0.1, INTAKE_IDLE_CURRENT, gamePieceSupplier);
    }
}
