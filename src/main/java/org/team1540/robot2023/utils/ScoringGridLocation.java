package org.team1540.robot2023.utils;




public final class ScoringGridLocation implements ScoringGrid.OuterGrid, ScoringGrid.InnerGrid {
    public static final OuterGrid TOP_GRID    = new ScoringGridLocation("TopGrid",    PolePosition.RIGHT, PolePosition.LEFT);
    public static final InnerGrid MIDDLE_GRID = new ScoringGridLocation("MiddleGrid", null, null);
    public static final OuterGrid BOTTOM_GRID = new ScoringGridLocation("BottomGrid", PolePosition.LEFT, PolePosition.RIGHT);

    private final String key;
    private final PolePosition outerPole;
    private final PolePosition innerPole;

    private ScoringGridLocation(String key, PolePosition outer, PolePosition inner) {
        this.key = key;
        this.outerPole = outer;
        innerPole = inner;
    }
    public String getPathName(String basename) {
        return key+basename;
    }

    public PolePosition getOuterPole() {
        return outerPole;
    }

    public PolePosition getInnerPole() {
        return innerPole;
    }
}
