package org.team1540.robot2023.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Encoder;

public class ChickEncoder extends Encoder {
    private final double pulsesPerRevolution;
    private double offsetDegrees;

    public ChickEncoder(int channelA, int channelB, double pulsesPerRevolution) {
        this(channelA, channelB, pulsesPerRevolution, false);
    }

    public ChickEncoder(int channelA, int channelB, double pulsesPerRevolution, boolean reverseDirection) {
        super(channelA, channelB, reverseDirection);
        this.pulsesPerRevolution = pulsesPerRevolution;
    }

    public void setPosition(double positionDegrees) {
        reset();
        offsetDegrees = positionDegrees;
    }

    public void setPosition(Rotation2d position) {
        reset();
        offsetDegrees = position.getDegrees();
    }

    public double getDegrees() {
        return (get() * 360)/pulsesPerRevolution + offsetDegrees;
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getDegrees());
    }
}
