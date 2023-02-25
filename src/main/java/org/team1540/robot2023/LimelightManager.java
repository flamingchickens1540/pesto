package org.team1540.robot2023;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import org.team1540.robot2023.utils.Limelight;

import java.util.LinkedList;

import static org.team1540.robot2023.Globals.field2d;

public class LimelightManager {
    private static LimelightManager instance;
    private final LinkedList<Limelight> limelights = new LinkedList<>();
    private LimelightManager() {}
    public static LimelightManager getInstance() {
        if (instance == null) {
            instance = new LimelightManager();
        }
        return instance;
    }

    public void addLimelight(Limelight limelight) {
        limelights.add(limelight);
        limelight.setLedState(Limelight.LEDMode.OFF);
        limelight.setPipeline(Limelight.Pipeline.APRIL_TAGS);
        limelight.setDriverMode(false);
    }
    public void addLimelight(String name) {
        addLimelight(new Limelight(name));
    }

    public void periodic() {
        for (Limelight limelight: limelights) {
            limelight.periodic();
            Pose2d rawPose = limelight.getBotPose();
            if (rawPose != null) {
                field2d.getObject("pose/" + limelight.name + "/unfiltered").setPose(rawPose);
            } else {
                field2d.getObject("pose/" + limelight.name + "/unfiltered").setPoses();
            }
        }
    }

    public void applyEstimates(SwerveDrivePoseEstimator poseEstimator) {
        for (Limelight limelight: limelights) {
            Pose2d pose = limelight.getFilteredBotPose();
            if (pose != null) {
                field2d.getObject("pose/"+limelight.name).setPose(pose);
                poseEstimator.addVisionMeasurement(pose, Timer.getFPGATimestamp()-(limelight.getDeltaTime()/1000));
            } else {
                // Remove poses if no target is seen
                field2d.getObject("pose/"+limelight.name).setPoses();
            }
        }
    }
}
