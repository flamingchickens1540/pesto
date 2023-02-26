package org.team1540.robot2023.commands.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.team1540.robot2023.utils.ArmState;

public class Arm {
    public final Pivot pivot = new Pivot();
    public final Telescope telescope = new Telescope(this);
    public Arm() {
        CommandScheduler.getInstance().getDefaultButtonLoop().bind(this::smashDartboard);
    }

    public double getMaxExtension() {
        return telescope.getMaxExtension(pivot.getRotation2d());
    }


    public ArmState getArmState() {
        return ArmState.fromRotationExtension(pivot.getRotation2d(), telescope.getExtension());
    }


    public void stopAll() {
        pivot.stop();
        telescope.stop();
    }

    private void smashDartboard() {
        SmartDashboard.putNumber("arm/Xpos", getArmState().getX());
        SmartDashboard.putNumber("arm/Ypos", getArmState().getY());
    }
}
