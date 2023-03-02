package org.team1540.robot2023.utils;

public class GridScoreData {
    public ArmState approach = null;
    public ArmState score = null;
    public ArmState retreat = null;
    public double additionalBackoff = 0;

    public PolePosition polePosition = PolePosition.CENTER;
    public GridScoreData(ArmState approach, ArmState score, ArmState retreat) {
        this.approach = approach;
        this.score = score;
        this.retreat = retreat;
    }
    public GridScoreData(ArmState approach, ArmState score, ArmState retreat, double additionalBackoff) {
        this.approach = approach;
        this.score = score;
        this.retreat = retreat;
        this.additionalBackoff = additionalBackoff;
    }
    public GridScoreData(ArmState position) {
        this.approach = position;
    }
    public GridScoreData(ArmState approach, ArmState score) {
        this.approach = approach;
        this.score = score;
    }

    public GridScoreData withRetreat(ArmState retreat) {
        GridScoreData data = copy();
        data.retreat = retreat;
        return data;
    }
    public GridScoreData withAdditionalBackoff(double backoff) {
        GridScoreData data = copy();
        data.additionalBackoff = backoff;
        return data;
    }
    public GridScoreData withScore(ArmState score) {
        GridScoreData data = copy();
        data.score = score;
        return data;
    }

    public GridScoreData withPolePosition(PolePosition polePosition) {
        GridScoreData data = copy();
        data.polePosition = polePosition;
        return data;
    }

    public GridScoreData copy() {
        GridScoreData data = new GridScoreData(null);
        data.approach = this.approach;
        data.score = this.score;
        data.retreat = this.retreat;
        data.additionalBackoff = this.additionalBackoff;
        data.polePosition = this.polePosition;
        return data;

    }
}
