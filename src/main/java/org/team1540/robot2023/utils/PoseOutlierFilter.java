package org.team1540.robot2023.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.LinkedList;

public class PoseOutlierFilter {
    private final LinkedList<Pose2d> items = new LinkedList<>();
    private final int count;
    private final LinkedList<Double> xVals = new LinkedList<>();
    private final LinkedList<Double> yVals = new LinkedList<>();


    public PoseOutlierFilter(int count) {
        this.count = count;

    }

    public void add(Translation2d item) {
        xVals.addLast(item.getX());
        yVals.addLast(item.getY());
        if (xVals.size() > count)  {
            xVals.removeFirst();
            yVals.removeFirst();
        }
        if (yVals.size() > count) {
            yVals.removeFirst();
        }
    }

    private double median(LinkedList<Double> list) {
        LinkedList<Double> copyList = new LinkedList<>(list);
        copyList.sort((a, b) -> (int) Math.signum(b - a));
        if (copyList.size() % 2 == 1) {
            return copyList.get(copyList.size()/2);
        } else {
            return (copyList.get(copyList.size()/2)+copyList.get(copyList.size()/2-1))/2;
        }
    }
    public Translation2d medianPose() {
        return new Translation2d(median(xVals), median(yVals));

    }
}
