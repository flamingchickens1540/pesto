package org.team1540.robot2023.commands.drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static org.team1540.robot2023.utils.MathUtils.deadzone;

import java.util.function.BooleanSupplier;

import org.team1540.robot2023.commands.grabber.WheeledGrabber;

public class SwerveDriveCommand extends CommandBase {
    private final Drivetrain drivetrain;
    private final XboxController controller;
    private final double deadzone = 0.1;
    private double xyscale = 1;
    private double rotscale = 1;

    // The rate limit should be relative to the input percent. A value of 1 will take 1 second to get from 0% to 100%, a value of 2 will do that in half a second
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(2);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(2);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);
    private final DoubleLogEntry xyscaleLog = new DoubleLogEntry(DataLogManager.getLog(),"CUSTOM:xyscale" );
    private final DoubleLogEntry rotscaleLog = new DoubleLogEntry(DataLogManager.getLog(),"CUSTOM:rotscale");
    private final BooleanSupplier forwardOnlySupplier;
    private final WheeledGrabber intake;

    public SwerveDriveCommand(Drivetrain drivetrain, XboxController controller, BooleanSupplier forwardOnlySupplier, WheeledGrabber intake) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        this.forwardOnlySupplier = forwardOnlySupplier;
        this.intake = intake;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        xLimiter.reset(0);
        yLimiter.reset(0);
        rotLimiter.reset(0);
    }

    @Override
    public void execute() {
//        if (controller.getXButton()) {
//            xyscale = 0.25;
//            rotscale = 0.25;
//        }
//        if (controller.getBButton()) {
//            xyscale = 1;
//            rotscale = 1;
//        }

        xyscaleLog.append(xyscale);
        rotscaleLog.append(rotscale);
        if (forwardOnlySupplier.getAsBoolean()) {
            drivetrain.drive(
                    xLimiter.calculate(Math.min(deadzone(-controller.getLeftY(), deadzone), intake.hasGamePiece()?0:1)*0.25),
                    0,
                    rotLimiter.calculate(-deadzone(controller.getRightX(), deadzone)*rotscale*0.25),
                    false
            );
        } else {
            drivetrain.drive(
                    xLimiter.calculate(deadzone(-controller.getLeftY(), deadzone)*xyscale),
                    yLimiter.calculate(deadzone(-controller.getLeftX(), deadzone)*xyscale),
                    rotLimiter.calculate(-deadzone(controller.getRightX(), deadzone)*rotscale),
                    true
            );
        }
    }

    @Override
    public void end(boolean isInterrupted) {
        drivetrain.drive(0, 0, 0, false);
    }
}