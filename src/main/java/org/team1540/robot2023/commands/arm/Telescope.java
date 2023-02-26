package org.team1540.robot2023.commands.arm;

import com.revrobotics.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.lib.math.Conversions;
import org.team1540.robot2023.Constants;

public class Telescope extends SubsystemBase {
    private final CANSparkMax telescope = new CANSparkMax(Constants.ArmConstants.TELESCOPE_ID,
            CANSparkMaxLowLevel.MotorType.kBrushless);
    private final RelativeEncoder telescopeEncoder = telescope.getEncoder();
    private final SparkMaxPIDController telescopePID = telescope.getPIDController();
    private final SparkMaxLimitSwitch telescopeLimitSwitch = telescope.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);


    private final Arm parent;
    public Telescope(Arm parent){
        this.parent = parent;
        telescope.setSmartCurrentLimit(40);
        telescope.setIdleMode(CANSparkMax.IdleMode.kBrake);

        telescope.setInverted(true);
        telescope.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, Constants.ArmConstants.TELESCOPE_FORWARD_LIMIT);
        telescope.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);

        telescopePID.setP(Constants.ArmConstants.TELESCOPE_KP);
        telescopePID.setI(Constants.ArmConstants.TELESCOPE_KI);
        telescopePID.setD(Constants.ArmConstants.TELESCOPE_KD);
        telescopePID.setSmartMotionMaxAccel(Constants.ArmConstants.TELESCOPE_MAX_ACCEL, 0);
        telescopePID.setSmartMotionMaxVelocity(Constants.ArmConstants.TELESCOPE_CRUISE_SPEED, 0);
        telescopePID.setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kTrapezoidal, 0);
    }

    double getExtension() {
        return telescopeEncoder.getPosition() * Constants.ArmConstants.EXT_ROTS_TO_INCHES / Constants.ArmConstants.EXT_GEAR_RATIO + Constants.ArmConstants.ARM_BASE_LENGTH;
    }

    public boolean getLimitSwitch() {
        return telescopeLimitSwitch.isPressed();
    }

    protected void setExtension(double extension) {
        telescopePID.setReference(
                (extension - Constants.ArmConstants.ARM_BASE_LENGTH) * Constants.ArmConstants.EXT_GEAR_RATIO / Constants.ArmConstants.EXT_ROTS_TO_INCHES,
                CANSparkMax.ControlType.kPosition, 0
        );

    }

    public void stop(){
        telescope.set(0);
    }

    public void setExtendingSpeed(double speed){
        telescope.set(speed);
    }

    public double getMaxExtension(Rotation2d rotation) {
        double angle = Conversions.actualToCartesian(rotation).getRadians();
        if (angle == Math.PI / 2) return Constants.ArmConstants.MAX_LEGAL_HEIGHT;
        if (angle == 0 || angle == Math.PI) return Constants.ArmConstants.MAX_LEGAL_DISTANCE;
        else if (angle > 0 && angle < Math.PI){
            return Math.min(Math.abs(Constants.ArmConstants.MAX_LEGAL_DISTANCE / Math.cos(angle)), Math.abs(Constants.ArmConstants.MAX_LEGAL_HEIGHT / Math.sin(angle)));
        }
        else {
            return Math.min(Math.abs(Constants.ArmConstants.MAX_LEGAL_DISTANCE / Math.cos(angle)), Math.abs(Constants.ArmConstants.PIVOT_HEIGHT / Math.sin(angle)));
        }
    }

    @Override
    public void periodic() {

        if (getLimitSwitch()) telescopeEncoder.setPosition(0);
        SmartDashboard.putNumber("arm/extension", getExtension());
        SmartDashboard.putBoolean("arm/limit", getLimitSwitch());
        SmartDashboard.putNumber("arm/extensionRots", telescopeEncoder.getPosition());
        SmartDashboard.putBoolean("arm/isLegal", getExtension() > parent.getMaxExtension());
    }
}
