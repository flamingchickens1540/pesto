package org.team1540.robot2023;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import org.team1540.robot2023.utils.Limelight;

import java.util.List;

import static org.team1540.robot2023.Globals.field2d;

public class LimelightManager {
    private static LimelightManager instance;

    public final Limelight frontLimelight = new Limelight("limelight-front");
    public final Limelight rearLimelight = new Limelight("limelight-rear");
    private final List<Limelight> limelights = List.of(frontLimelight, rearLimelight);
    private LimelightManager() {
        for(Limelight limelight: limelights) {
            limelight.setLedState(Limelight.LEDMode.OFF);
            limelight.setPipeline(Limelight.Pipeline.APRIL_TAGS);
            limelight.setDriverMode(false);
        }
    }

    public static LimelightManager getInstance() {
        if (instance == null) {
            instance = new LimelightManager();
        }
        return instance;
    }


    public void periodic() {
        for (Limelight limelight: limelights) {
            limelight.periodic();
//            Pose2d rawPose = limelight.getBotPose();
//            if (rawPose != null) {
//                field2d.getObject("pose/" + limelight.name + "/unfiltered").setPose(rawPose);
//            } else {
//                field2d.getObject("pose/" + limelight.name + "/unfiltered").setPoses();
//            }
        }
    }

    public boolean applyFrontEstimates(SwerveDrivePoseEstimator poseEstimator, Rotation2d gyroAngle, SwerveModulePosition[] positions) {
        frontLimelight.setPipeline(Limelight.Pipeline.APRIL_TAGS);
        Pose2d pose = frontLimelight.getBotPose();
        if (pose != null) {
            field2d.getObject("pose/"+frontLimelight.name).setPose(pose);
//                poseEstimator.addVisionMeasurement(pose, Timer.getFPGATimestamp()-(limelight.getDeltaTime()/1000));
            poseEstimator.resetPosition(gyroAngle, positions, pose);
            return true;
        } else {
            // Remove poses if no target is seen

            field2d.getObject("pose/"+frontLimelight.name).setPoses();
        }
        return false;
    }

    public boolean zeroFromLimelights(SwerveDrivePoseEstimator poseEstimator, Rotation2d gyroAngle, SwerveModulePosition[] positions) {
        for (Limelight limelight: limelights) {
            limelight.setPipeline(Limelight.Pipeline.APRIL_TAGS);
            Pose2d pose = limelight.getBotPose();
            if (pose != null) {
                field2d.getObject("pose/"+limelight.name).setPose(pose);
//                poseEstimator.addVisionMeasurement(pose, Timer.getFPGATimestamp()-(limelight.getDeltaTime()/1000));
                poseEstimator.resetPosition(gyroAngle, positions, pose);
                return true;
            } else {
                // Remove poses if no target is seen

                field2d.getObject("pose/"+limelight.name).setPoses();
            }
        }
        return false;
    }

    public boolean canSeeTargets() {
        for (Limelight limelight: limelights) {
            Pose2d pose = limelight.getFilteredBotPose();
            if (pose != null) {
                return true;
            }
        }
        return false;
    }
}
