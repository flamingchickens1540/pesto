package org.team1540.robot2023.utils;

import edu.wpi.first.math.util.Units;

import static org.team1540.robot2023.Constants.poleOffsetMeters;

public enum PolePosition {
    LEFT(-poleOffsetMeters),
    RIGHT(poleOffsetMeters),
    CENTER(0);

    public final double offset;
    PolePosition(double meters) {
        this.offset = meters;
    }
}
