package org.team1540.robot2023.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.LinkedList;
import java.util.Optional;

public class PoseMedianFilter {
    private final LinkedList<Pose2d> items = new LinkedList<>();
    private final int count;
    private final LinkedList<Double> xVals = new LinkedList<>();
    private final LinkedList<Double> yVals = new LinkedList<>();


    public PoseMedianFilter(int count) {
        this.count = count;

    }

    public void add(Translation2d item) {
        xVals.addLast(item.getX());
        yVals.addLast(item.getY());
        if (xVals.size() > count)  {
            xVals.removeFirst();
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
    public Optional<Pose2d> checkOutlier(Pose2d pose) {
        Translation2d median = new Translation2d(median(xVals), median(yVals));
        Translation2d diff = pose.getTranslation().minus(median);
        if (diff.getNorm()>0.16*count/2) {
            return Optional.empty();
        } else {
            return Optional.of(pose);
        }


    }
}
