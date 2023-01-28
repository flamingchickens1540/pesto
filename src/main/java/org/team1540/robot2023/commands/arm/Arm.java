package org.team1540.robot2023.commands.arm;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2023.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    private final TalonFX pivot1 = new TalonFX(ArmConstants.PIVOT1_ID);
    private final TalonFX pivot2 = new TalonFX(ArmConstants.PIVOT2_ID);
    private final TalonFX telescope = new TalonFX(ArmConstants.TELESCOPE_ID);
    private final CANCoder cancoder = new CANCoder(ArmConstants.CANCODER_ID);

    private double extensionSetPoint = 0;
    private boolean notSet = false;

    private boolean extending = false;


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
        double theta = getAngleRadians();
        return theta>=0?Math.min(ArmConstants.MAX_DISTANCE / Math.cos(theta), ArmConstants.MAX_HEIGHT / Math.sin(theta))
                : Math.min(ArmConstants.MAX_DISTANCE / Math.cos(theta), -ArmConstants.PIVOT_HEIGHT / Math.sin(theta));
    }

    public double getAngleRadians() {
        // TODO: figure out cancoder stuff for this
        //We need to set up the cancoder so that it has the correct sensor direction, boot initialization,
        //-180 to 180 sensor range, and correct offset
        return Math.toRadians(cancoder.getPosition());
    }

    public double getExtension() {
        // TODO: figure this out
        return 0;
    }

    public void setAngleRadians(double angle) {
        // TODO: something
        //Feedforward needs to incorporate how extended the arm is
        //We should also try calculating kF instead of arbitrary
        double feedforward = Math.cos(getAngleRadians())*ArmConstants.PIVOT_FF;
        pivot1.set(ControlMode.MotionMagic, angle, DemandType.ArbitraryFeedForward, feedforward);
    }

    public void setExtensionSetPoint(double extensionSetPoint) {
        this.extensionSetPoint = extensionSetPoint;
        setExtension(extensionSetPoint);
        extending = true;
    }

    private void setExtension(double extension) {
        // TODO: magic
        //Talk to Kevin about a feedforward for this
        //Might need to be something similar to an arm but with max at straight up not straight out
        //Or in other words, sin
        double feedforward = Math.sin(getAngleRadians())*ArmConstants.TELESCOPE_FF;
        telescope.set(ControlMode.Position,extension, DemandType.ArbitraryFeedForward, feedforward);
//        telescope.set(ControlMode.Position,extension);

    }

    public void stopAll() {
        pivot1.set(ControlMode.PercentOutput, 0);
        telescope.set(ControlMode.PercentOutput, 0);
        extending = false;
    }

    private void limitArmExtension(){
        // TODO: 1/28/2023 Keep an eye on this if problems arise
        if(getMaxExtension() < getExtension()){
            setExtension(getMaxExtension());
            notSet = true;
        }
        else if(isExtending()){
            if(getMaxExtension() < extensionSetPoint){
                setExtension(getMaxExtension());
                notSet = true;
            }
            else if(notSet){
                setExtension(extensionSetPoint);
                notSet = false;
            }
        }
    }

    public boolean isRotating(){
        return pivot1.getSelectedSensorVelocity() > 0.1;
    }

    public boolean isExtending(){
        return telescope.getSelectedSensorVelocity() > 0.1;
    }

    @Override
    public void periodic() {
        limitArmExtension();
    }
}
