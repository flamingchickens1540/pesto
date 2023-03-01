package org.team1540.robot2023.utils;

public class GridScoreData {
    public ArmState approach = null;
    public ArmState score = null;
    public ArmState retreat = null;
    public GridScoreData(ArmState approach, ArmState score, ArmState retreat) {
        this.approach = approach;
        this.score = score;
        this.retreat = retreat;
    }
    public GridScoreData(ArmState position) {
        this.approach = position;
    }
    public GridScoreData(ArmState approach, ArmState score) {
        this.approach = approach;
        this.score = score;
    }
}
