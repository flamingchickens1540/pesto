package org.team1540.robot2023.utils;

import java.util.Arrays;
import java.util.LinkedList;

public class PoseZeroFilter {
    private final LinkedList<Boolean> items = new LinkedList<>();
    private final int count;
    private final int threshold;

    private int nonZero;

    /**
     * Creates a new PoseZeroFilter
     * @param count The number of elements to keep track of
     * @param threshold The number of elements that must be non-zero to pass the filter
     */
    public PoseZeroFilter(int count, int threshold) {
        this.count = count;
        this.threshold = threshold;

    }

    public void add(double[] item) {
        if (!Arrays.equals(item, new double[item.length])) {
            nonZero++;
            items.addLast(true);
        } else {
            items.addLast(false);
        }

        if (items.size() > count) {
            nonZero -= items.removeFirst() ? 1 : 0;
        }
    }

    public boolean isNonZero() {
        return nonZero >= threshold;
    }
}
