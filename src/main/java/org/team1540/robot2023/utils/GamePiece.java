package org.team1540.robot2023.utils;

import org.team1540.lib.RevBlinkin;

public enum GamePiece {
    CONE("cone", RevBlinkin.ColorPattern.YELLOW, 1),
    CUBE("cube", RevBlinkin.ColorPattern.VIOLET, -1);

    public final String identifier;
    public final RevBlinkin.ColorPattern pattern;
    public final int intakeMultiplier;

    GamePiece(String identifier, RevBlinkin.ColorPattern pattern, int intakeMultiplier) {
        this.pattern = pattern;
        this.identifier = identifier;
        this.intakeMultiplier = intakeMultiplier;
    }
}
