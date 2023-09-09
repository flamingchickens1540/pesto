package org.team1540.robot2023.commands.wristintake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2023.utils.GamePiece;

import java.util.function.Supplier;

public abstract class IntakeCommandBase extends CommandBase {
    protected final RollerIntake intake;
    protected final Supplier<GamePiece> gamePieceSupplier;
    protected final double absPercentOutput;
    protected final double currentLimit;

    private GamePiece currentGamePiece;

    /**
     * @param intake Intake subsystem
     * @param absPercentOutput Percent output of intake, regardless of game piece. Positive for intake, negative for outtake
     * @param currentLimit Current limit
     * @param gamePieceSupplier Supplier for current game piece mode of robot
     */
    public IntakeCommandBase(RollerIntake intake, double absPercentOutput, double currentLimit, Supplier<GamePiece> gamePieceSupplier) {
        this.intake = intake;
        this.absPercentOutput = absPercentOutput;
        this.gamePieceSupplier = gamePieceSupplier;
        this.currentLimit = currentLimit;
    }

    @Override
    public void initialize() {
        currentGamePiece = gamePieceSupplier.get();
        intake.setCurrentLimit(currentLimit);
        intake.setSpeed(currentGamePiece.intakeMultiplier * absPercentOutput);
    }

    @Override
    public void execute() {
        if (!currentGamePiece.equals(gamePieceSupplier.get())) {
            currentGamePiece = gamePieceSupplier.get();
            intake.setSpeed(currentGamePiece.intakeMultiplier * absPercentOutput);
        }
    }
}
