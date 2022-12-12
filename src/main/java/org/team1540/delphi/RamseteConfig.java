package org.team1540.delphi;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;



public class RamseteConfig {
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.2;

    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(Constants.DRIVETRAIN_TRACKWIDTH_METERS);

    public static final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                    Constants.KS_VOLTS,
                    Constants.KV_VOLT_SECONDS_PER_METER,
                    Constants.KA_VOLT_SECONDS_SQUARED_PER_METER
            ),
            kDriveKinematics,
            10
    );

    public static final TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared
    ).setKinematics(kDriveKinematics)
            .addConstraint(autoVoltageConstraint);

    public static final RamseteController ramseteController = new RamseteController(kRamseteB, kRamseteZeta);
    public static final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(Constants.KS_VOLTS, Constants.KV_VOLT_SECONDS_PER_METER, Constants.KA_VOLT_SECONDS_SQUARED_PER_METER);
}