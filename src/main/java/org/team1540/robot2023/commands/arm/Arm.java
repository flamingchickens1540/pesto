package org.team1540.robot2023.commands.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2023.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    private final TalonFX pivot1 = new TalonFX(ArmConstants.PIVOT1_ID);
    private final TalonFX pivot2 = new TalonFX(ArmConstants.PIVOT2_ID);
    private final TalonFX telescope = new TalonFX(ArmConstants.TELESCOPE_ID);
    private final CANCoder cancoder = new CANCoder(ArmConstants.CANCODER_ID);

    public Arm() {
        pivot1.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, 0));
        pivot2.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, 0));
        telescope.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, 0));

        pivot1.setNeutralMode(NeutralMode.Brake);
        pivot2.setNeutralMode(NeutralMode.Brake);
        telescope.setNeutralMode(NeutralMode.Brake);

        pivot2.setInverted(true);
        pivot2.follow(pivot1);

        pivot1.config_kP(0, ArmConstants.PIVOT_KP);
        pivot1.config_kI(0, ArmConstants.PIVOT_KI);
        pivot1.config_kD(0, ArmConstants.PIVOT_KD);

        telescope.config_kP(0, ArmConstants.TELESCOPE_KP);
        telescope.config_kI(0, ArmConstants.TELESCOPE_KI);
        telescope.config_kD(0, ArmConstants.TELESCOPE_KD);
    }

    public double getMaxExtension() {
        double maxX = 48 + ArmConstants.PIVOT_DISTANCE;
        double maxY = 78 - ArmConstants.PIVOT_HEIGHT;
        double theta = getAngleRadians();
        return Math.min(maxX / Math.cos(theta), maxY / Math.sin(theta));
    }

    public double getAngleRadians() {
        // TODO: figure out cancoder stuff for this
        return 0;
    }

    public double getExtension() {
        // TODO: figure this out
        return 0;
    }

    public void setAngleRadians(double angle) {
        // TODO: something
    }

    public void setExtension(double extension) {
        // TODO: magic
    }

    public void stopAll() {
        pivot1.set(ControlMode.PercentOutput, 0);
        telescope.set(ControlMode.PercentOutput, 0);
    }
}
