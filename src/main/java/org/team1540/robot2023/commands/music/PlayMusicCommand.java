package org.team1540.robot2023.commands.music;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.team1540.robot2023.commands.arm.Arm;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;

public class PlayMusicCommand extends SequentialCommandGroup {
    public PlayMusicCommand(String songTitle, MusicPlayer player, Arm arm, Drivetrain drivetrain) {
        addRequirements(arm, drivetrain);
        addCommands(
                new InstantCommand(() -> player.loadMusic(songTitle)).andThen(new WaitCommand(0.1)), //Don't know how long loading music takes, waits just to be safe
                new InstantCommand(player::play),
                new WaitUntilCommand(() -> !player.isPlaying())
        );
    }
}
