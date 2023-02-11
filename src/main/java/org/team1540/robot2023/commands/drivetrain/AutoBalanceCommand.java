package org.team1540.robot2023.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2023.utils.AverageFilter;

import java.util.function.Supplier;

public class AutoBalanceCommand extends CommandBase {
    private final Drivetrain drivetrain;
    private final PIDController pidController = new PIDController(1, 0, 0);
    private final boolean isPitch;
    private final Supplier<Rotation2d> angleSupplier;
    private final AverageFilter filter = new AverageFilter(200);

    public AutoBalanceCommand(Drivetrain drivetrain, boolean isPitch) {
        this.isPitch = isPitch;
        angleSupplier = isPitch ? drivetrain::getPitch : drivetrain::getRoll;
        SmartDashboard.setDefaultNumber("balancy/kp", 0.0075);
        SmartDashboard.setDefaultNumber("balancy/ki", 0);
        SmartDashboard.setDefaultNumber("balancy/kd", 0.002);
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
    }

    @Override
    public void execute() {
        pidController.setPID(
                SmartDashboard.getNumber("balancy/kp", 0),
                SmartDashboard.getNumber("balancy/ki", 0),
                SmartDashboard.getNumber("balancy/kd", 0)
        );
        SmartDashboard.putNumber("balancy/output", pidController.getPositionError());
        filter.add(pidController.getPositionError());
        if (isPitch) {
            drivetrain.drive(
                    pidController.calculate(angleSupplier.get().getDegrees()), 0, 0, false);
        } else {
            drivetrain.drive(
                    0, -pidController.calculate(angleSupplier.get().getDegrees()), 0, false);
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(filter.getAverage()) < 1;

    }

    @Override
    public void end(boolean isInterrupted) {
        filter.clear();
    }


}
