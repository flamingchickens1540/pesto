package org.team1540.delphi.commands.drivetrain;

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

public class SwerveModule {
    private static final double kWheelRadius = 1.75; // 0.0508
    private static final int kEncoderResolution = 4096; // Ticks per rotation
    
    private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
    private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared
    
    private final TalonFX driveMotor;
    private final TalonFX turningMotor;
    
    private final CANCoder canCoder;
    
    private final PIDController drivePIDController = new PIDController(1, 0, 0); // Gain needs to be calculated for robot
    
    private final ProfiledPIDController turningPIDController = new ProfiledPIDController(
        kModuleMaxAngularVelocity,
        kModuleMaxAngularAcceleration,
        kEncoderResolution, 
        null
    );
    
    public SwerveModule(int moduleID, ModuleOffset offset, ModulePosition position) {
        driveMotor = new TalonFX(SwerveCANDevice.getDrivingMotorID(moduleID), Constants.CANIVORE_NAME);
        turningMotor = new TalonFX(SwerveCANDevice.getTurningMotorID(moduleID), Constants.CANIVORE_NAME);
        canCoder = new CANCoder(SwerveCANDevice.getCancoderID(moduleID), Constants.CANIVORE_NAME);
        
        CANCoderConfig.applyConfig(offset.getAs(position), canCoder);
        
        turningMotor.setSelectedSensorPosition(canCoder.getAbsolutePosition());
        
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }
    
    /**
     * Gets the current state of the module
     * @return The current velocity and angle as a SwerveModuleState
     */
    public SwerveModuleState getPosition() {
        return new SwerveModuleState(
            driveMotor.getSelectedSensorVelocity(),
            Rotation2d.fromDegrees(turningMotor.getSelectedSensorPosition())
        );
    }
    
    /**
     * Adjust the module's angle and speed to match the desiredState 
     * @param desiredState Desired state with speed and angle. (sets desired state for module )
    */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(turningMotor.getSelectedSensorPosition()));
        
        double driveOutput = drivePIDController.calculate(driveMotor.getSelectedSensorVelocity(), state.speedMetersPerSecond); 
        double turnOutput = turningPIDController.calculate(turningMotor.getSelectedSensorVelocity(), state.angle.getRadians()); 
        
        driveMotor.set(ControlMode.PercentOutput, driveOutput);
        turningMotor.set(ControlMode.PercentOutput, turnOutput);
    }
    
}
