package org.team1540.robot2023.commands.wristintake;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.team1540.lib.RevBlinkin;
import org.team1540.robot2023.utils.BlinkinManager;
import org.team1540.robot2023.utils.GamePiece;

import java.util.function.Supplier;

import static org.team1540.robot2023.Constants.RollerIntakeConstants.*;

public class IntakeIntakeCommand extends IntakeCommandBase {

    public IntakeIntakeCommand(RollerIntake intake, Supplier<GamePiece> gamePieceSupplier) {
        super(intake, 1, INTAKE_IDLE_CURRENT, gamePieceSupplier);
    }

    @Override
    public boolean isFinished() {
        return intake.hasGamePiece();
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            Commands.sequence(
                    BlinkinManager.getInstance().commandSet(RevBlinkin.ColorPattern.STROBE_BLUE),
                    new WaitCommand(1),
                    BlinkinManager.getInstance().commandSet(BlinkinManager.ColorPair.TELEOP)
            ).schedule();
        }
    }
}
