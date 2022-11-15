package org.team1540.delphi.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeCommand extends CommandBase {
    private final Intake intake;

    public IntakeCommand(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void initialize() {
        intake.setPercent(0.3);
    }

    @Override
    public void end(boolean isInterrupted) {
        intake.setPercent(0);
    }


}
