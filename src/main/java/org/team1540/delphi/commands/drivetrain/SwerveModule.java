package org.team1540.delphi.commands.drivetrain;


import org.team1540.delphi.utils.swerve.CANCoderConfig;
import org.team1540.delphi.utils.swerve.ModuleOffset;
import org.team1540.delphi.utils.swerve.ModulePosition;
import org.team1540.delphi.utils.swerve.SwerveCANDevice;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;



public class SwerveModule {
    private static final double kWheelRadius = 1.75; //0.0508
    private static final int kEncoderResolution = 4096; // Ticks per rotation

    private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
    private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    private final CANCoder canCoder;


    //private final PIDController m_drivePIDController = new PIDController(1, 0, 0); //Gain needs to be calculated for robot 
    
    public SwerveModule(int moduleID, ModuleOffset offset, ModulePosition position) { //can coder, 2 motors per moduel 
        driveMotor = new TalonFX(SwerveCANDevice.DRIVING_MOTOR.getDeviceID(moduleID));
        turningMotor = new TalonFX(SwerveCANDevice.TURNING_MOTOR.getDeviceID(moduleID));
        
        canCoder = new CANCoder(SwerveCANDevice.CANCODER.getDeviceID(moduleID));
        CANCoderConfig.applyConfig(offset.getAs(position), canCoder);

        turningMotor.setSelectedSensorPosition(canCoder.getAbsolutePosition());
    }
    /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
    public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(turningMotor.getSelectedSensorPosition()));

    // Calculate the drive output from the drive PID controller.
    //     driveMotor.config_kpd

    //     final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // // Calculate the turning motor output from the turning PID controller.
    //     final double turnOutput =
    //         m_turningPIDController.calculate(m_turningEncoder.get(), state.angle.getRadians());

    //     final double turnFeedforward =
    //         m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    //     driveMotor.set(ControlMode.)
    //     turningMotor.setVoltage(turnOutput + turnFeedforward);
  }
  public SwerveModuleState getPosition() {
    return new SwerveModuleState(driveMotor.getSelectedSensorVelocity(), Rotation2d.fromDegrees(turningMotor.getSelectedSensorPosition()));
  }





}
