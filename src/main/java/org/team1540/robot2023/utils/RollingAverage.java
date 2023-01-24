package org.team1540.robot2023.utils;

import java.util.LinkedList;

public class RollingAverage {

    private final LinkedList<Double> buffer;
    private final int size;
    private double sum;

    public RollingAverage(int size) {
        this.size = size;
        this.buffer = new LinkedList<>();
    }

    public void add(double num) {
        sum += num;
        buffer.add(num);
        if (buffer.size() > size) sum -= buffer.remove(0);
    }

    public double getAverage() {
        return sum / buffer.size();
    }

    public double getAverageAbs(){
        return Math.abs(getAverage());
    }

    public void clear() {
        buffer.clear();
        sum = 0;
    }
}
