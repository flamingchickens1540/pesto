package org.team1540.delphi.commands.drivetrain;

import com.swervedrivespecialties.swervelib.*;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import org.team1540.delphi.utils.swerve.CANCoderConfig;
import org.team1540.delphi.utils.swerve.ModuleOffset;
import org.team1540.delphi.utils.swerve.ModulePosition;
import org.team1540.delphi.utils.swerve.SwerveCANDevice;
import org.team1540.delphi.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import static org.team1540.delphi.Constants.*;

public class ChickenSwerveModule {
    private final SwerveModule module;
    public static final Mk4ModuleConfiguration config = new Mk4ModuleConfiguration();
    public static final double MAX_VELOCITY_METERS_PER_SECOND  = 6380.0 / 60.0 *
            SdsModuleConfigurations.MK4I_L1.getDriveReduction() *
            SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI;; // 3 meters per second
    public static final double kMaxAngularSpeed = MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);; // 1/2 rotation per second

    public ChickenSwerveModule(int moduleID, ModuleOffset offset, ModulePosition position) {
        config.setCanivoreName(CANIVORE_NAME);
//        throw new NullPointerException(config.toString());
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        module = Mk4iSwerveModuleHelper.createFalcon500(
                // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
//                tab.getLayout(position.label+" Module", BuiltInLayouts.kList)
//                        .withSize(2, 4)
//                        .withPosition(0, 0),
                config,
                // This can either be STANDARD or FAST depending on your gear configuration
                Mk4iSwerveModuleHelper.GearRatio.L1,
                // This is the ID of the drive motor
                SwerveCANDevice.getDrivingMotorID(moduleID),
                // This is the ID of the steer motor
                SwerveCANDevice.getTurningMotorID(moduleID),
                // This is the ID of the steer encoder
                SwerveCANDevice.getCancoderID(moduleID),
                // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
                offset.getAs(position)
        );
//        throw new NullPointerException(config.getCanivoreName());

    }

    public void set(SwerveModuleState state) {
        module.set(state.speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * 12, state.angle.getRadians());
    }

}
