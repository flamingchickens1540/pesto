package org.team1540.robot2023.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.team1540.lib.RevBlinkin;
import org.team1540.lib.RevBlinkin.ColorPattern;
public class BlinkinPair {
    private final RevBlinkin frontBlinkin;
    private final RevBlinkin rearBlinkin;
    public BlinkinPair(RevBlinkin front, RevBlinkin rear) {
        this.frontBlinkin = front;
        this.rearBlinkin = rear;
    }

    public Command commandSet(ColorPair pattern) {
        return new InstantCommand(() -> this.set(pattern)).ignoringDisable(true).withName("SetBlinkins");
    }
    public void set(ColorPair pair) {
        frontBlinkin.setPattern(pair.front);
        rearBlinkin.setPattern(pair.rear);
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
        CONE(ColorPattern.ORANGE, ColorPattern.WAVES_FOREST),
        CUBE(ColorPattern.VIOLET, ColorPattern.WAVES_FOREST);

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
