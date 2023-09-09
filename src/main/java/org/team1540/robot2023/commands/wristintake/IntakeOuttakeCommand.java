package org.team1540.robot2023.commands.wristintake;

import org.team1540.robot2023.utils.GamePiece;

import java.util.function.Supplier;

import static org.team1540.robot2023.Constants.RollerIntakeConstants.INTAKE_IDLE_CURRENT;

public class IntakeOuttakeCommand extends IntakeCommandBase {

    public IntakeOuttakeCommand(RollerIntake intake, double outtakeSpeed, Supplier<GamePiece> gamePieceSupplier) {
        super(intake, -Math.abs(outtakeSpeed), INTAKE_IDLE_CURRENT, gamePieceSupplier);
    }
}
