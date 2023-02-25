package org.team1540.robot2023;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.IOException;

public class Globals {
    public static final AprilTagFieldLayout aprilTagLayout;
    public static final Field2d field2d = new Field2d();
    static {
        SmartDashboard.putData("field", field2d);
        try {
            aprilTagLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

    }
}
