package org.team1540.robot2023.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.team1540.lib.RevBlinkin;
import org.team1540.lib.RevBlinkin.ColorPattern;

public class BlinkinPair {
    private final RevBlinkin frontBlinkin;
    private final RevBlinkin rearBlinkin;
    private ColorPattern gamePieceStyle = null;
    public BlinkinPair(RevBlinkin front, RevBlinkin rear) {
        this.frontBlinkin = front;
        this.rearBlinkin = rear;
    }

    public Command commandSet(ColorPair pattern) {
        return new InstantCommand(() -> this.set(pattern)).ignoringDisable(true).withName("SetBlinkins");
    }
    public void set(ColorPair pair) {
        if (pair == ColorPair.TELEOP && gamePieceStyle != null) {
            frontBlinkin.setPattern(gamePieceStyle);
        } else {
            frontBlinkin.setPattern(pair.front);
        }
        rearBlinkin.setPattern(pair.rear);
    }

    public void setGamepiece(boolean isCone) {
        gamePieceStyle = isCone ? ColorPattern.ORANGE : ColorPattern.VIOLET;
        set(ColorPair.TELEOP);
    }
    public InstantCommand commandSetGamepiece(boolean isCone) {
        return new InstantCommand(() -> {setGamepiece(isCone);});
    }

    public Command commandSet(ColorPattern pattern) {
        return new InstantCommand(() -> this.set(pattern)).ignoringDisable(true).withName("SetBlinkins");
    }
    public void set(ColorPattern pattern) {

        frontBlinkin.setPattern(pattern);
        rearBlinkin.setPattern(pattern);
    }

    public Command commandSet(ColorPattern front, ColorPattern rear) {
        return new InstantCommand(() -> this.set(front, rear)).ignoringDisable(true).withName("SetBlinkins");
    }
    public void set(ColorPattern front,ColorPattern rear) {
        frontBlinkin.setPattern(front);
        rearBlinkin.setPattern(rear);
    }

    public enum ColorPair {
        TELEOP(ColorPattern.FIRE_MEDIUM, ColorPattern.WAVES_FOREST),
        AUTO(ColorPattern.WAVES_PARTY, ColorPattern.WAVES_PARTY),
        APRILTAG(ColorPattern.DARK_RED, ColorPattern.WAVES_LAVA);

        public final ColorPattern front;
        public final ColorPattern rear;

        ColorPair(ColorPattern pattern) {
            this.front = pattern;
            this.rear = pattern;
        }
        ColorPair(ColorPattern front, ColorPattern rear) {
            this.front = front;
            this.rear = rear;
        }


    }
}
