package org.team1540.robot2023.utils;


// OK, please don't look to closely at this I swear this is how java classes are meant to be used don't worry about it
public interface ScoringGrid {
    String getPathName(String basename);
    interface OuterGrid extends ScoringGrid {
        PolePosition getOuterPole();
        PolePosition getInnerPole();
    }
    interface InnerGrid extends ScoringGrid {}
}
