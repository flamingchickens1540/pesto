package org.team1540.robot2023.utils;

import org.team1540.lib.RevBlinkin;
import org.team1540.lib.RevBlinkin.ColorPattern;
public class BlinkinPair {
    private final RevBlinkin frontBlinkin;
    private final RevBlinkin rearBlinkin;
    public BlinkinPair(RevBlinkin front, RevBlinkin rear) {
        this.frontBlinkin = front;
        this.rearBlinkin = rear;
    }

    public void set(ColorPair pair) {
        frontBlinkin.setPattern(pair.front);
        rearBlinkin.setPattern(pair.rear);
    }
    public void set(ColorPattern pattern) {
        frontBlinkin.setPattern(pattern);
        rearBlinkin.setPattern(pattern);
    }
    public void set(ColorPattern front,ColorPattern rear) {
        frontBlinkin.setPattern(front);
        rearBlinkin.setPattern(rear);
    }

    public enum ColorPair {
        TELEOP(ColorPattern.GREEN, ColorPattern.HOT_PINK),
        CONE(ColorPattern.YELLOW, ColorPattern.ORANGE),
        CUBE(ColorPattern.VIOLET, ColorPattern.BLUE);

        public final ColorPattern front;
        public final ColorPattern rear;

        ColorPair(ColorPattern pattern) {
            this.front = pattern;
            this.rear = pattern;
        }
        ColorPair(ColorPattern front, ColorPattern rear) {

            this.front = front;
            this.rear = rear;
        };

    }
}
