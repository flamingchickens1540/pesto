package org.team1540.commands.drivetrain;
import org.team1540.robotTemplate.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.sensors.CANCoder;



public class SwerveModule {
    private static final double kWheelRadius = 1.75; //0.0508
    private static final int kEncoderResolution = 4096; // Ticks per rotation

    private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
    private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

    private final MotorController m_driveMotor;
    private final MotorController m_turningMotor;
      
    private final Encoder m_driveEncoder;
    private final Encoder m_turningEncoder;

    //private final PIDController m_drivePIDController = new PIDController(1, 0, 0); //Gain needs to be calculated for robot 
    

    public SwerveModule(int moduleID) { //can coder, 2 motors per moduel 
        m_driveMotor = new Falcon(driveMotorChannel);
        m_turningMotor = new Falcon(turningMotorChannel);

      
        m_driveEncoder = new Encoder(driveEncoderChannelA, driveEncoderChannelB); 
        m_turningEncoder = new Encoder(turningEncoderChannelA, turningEncoderChannelB); 

        m_driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

        m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

        //m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }
     /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
    public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state =
            SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.get()));

    // Calculate the drive output from the drive PID controller.
        final double driveOutput =
            m_drivePIDController.calculate(m_driveEncoder.getRate(), state.speedMetersPerSecond);

        final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
        final double turnOutput =
            m_turningPIDController.calculate(m_turningEncoder.get(), state.angle.getRadians());

        final double turnFeedforward =
            m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

        m_driveMotor.setVoltage(driveOutput + driveFeedforward);
        m_turningMotor.setVoltage(turnOutput + turnFeedforward);
  }





}
