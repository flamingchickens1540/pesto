package org.team1540.robot2023.utils;

import static org.team1540.robot2023.Constants.Auto.gridPoleOffsetMeters;

public enum PolePosition {
    LEFT(-gridPoleOffsetMeters),
    RIGHT(gridPoleOffsetMeters),
    CENTER(0);

    public final double offset;
    PolePosition(double meters) {
        this.offset = meters;
    }
}
