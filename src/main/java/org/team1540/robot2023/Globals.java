package org.team1540.robot2023;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import java.io.IOException;

public class Globals {
    public static final AprilTagFieldLayout aprilTagLayout;

    static {
        try {
            aprilTagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}
