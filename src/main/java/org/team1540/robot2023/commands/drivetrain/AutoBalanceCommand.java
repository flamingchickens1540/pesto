package org.team1540.robot2023.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2023.utils.AverageFilter;

import java.util.function.Supplier;

public class AutoBalanceCommand extends CommandBase {
    private final Drivetrain drivetrain;
    private final PIDController pidController = new PIDController(0.01, 0, 0.002);
    private final Supplier<Rotation2d> angleSupplier;
    private final AverageFilter filter = new AverageFilter(200);

    public AutoBalanceCommand(Drivetrain drivetrain) {
        angleSupplier = drivetrain::getRoll;
        SmartDashboard.putData("balancy", pidController);


        this.drivetrain = drivetrain;
    }

    @Override
    public void execute() {
        addRequirements(drivetrain);
        SmartDashboard.putNumber("balancy/output", pidController.getPositionError());
        filter.add(pidController.getPositionError());
            drivetrain.drive(
                     pidController.calculate(angleSupplier.get().getDegrees()),0, 0, false);
    }

    @Override
    public boolean isFinished() {
        return false;
//        return Math.abs(filter.getAverage()) < 1;

    }

    @Override
    public void end(boolean isInterrupted) {
        filter.clear();
    }


}
