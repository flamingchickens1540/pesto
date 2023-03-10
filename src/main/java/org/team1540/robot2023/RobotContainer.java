package org.team1540.robot2023;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team1540.lib.RevBlinkin;
import org.team1540.robot2023.commands.arm.*;
import org.team1540.robot2023.commands.auto.*;
import org.team1540.robot2023.commands.drivetrain.Drivetrain;
import org.team1540.robot2023.commands.drivetrain.SwerveDriveCommand;
import org.team1540.robot2023.commands.grabber.DefaultGrabberCommand;
import org.team1540.robot2023.commands.grabber.GrabberIntakeCommand;
import org.team1540.robot2023.commands.grabber.GrabberOuttakeCommand;
import org.team1540.robot2023.commands.grabber.WheeledGrabber;
import org.team1540.robot2023.utils.BlinkinPair;
import org.team1540.robot2023.utils.ButtonPanel;
import org.team1540.robot2023.utils.PolePosition;
import org.team1540.robot2023.utils.ScoringGridLocation;

import java.util.concurrent.atomic.AtomicReference;

import static org.team1540.robot2023.Constants.ENABLE_PNEUMATICS;

public class RobotContainer {
    // Hardware

    AHRS gyro = new AHRS(SPI.Port.kMXP);
    RevBlinkin frontBlinken = new RevBlinkin(1, RevBlinkin.ColorPattern.WAVES_PARTY);
    RevBlinkin rearBlinken = new RevBlinkin(0, RevBlinkin.ColorPattern.WAVES_FOREST);
    BlinkinPair blinkins = new BlinkinPair(frontBlinken, rearBlinken);
    public final PneumaticHub ph = new PneumaticHub(Constants.PNEUMATIC_HUB);
    public final PowerDistribution pdh = new PowerDistribution(Constants.PDH, PowerDistribution.ModuleType.kRev);
    // Subsystems

    Drivetrain drivetrain = new Drivetrain(gyro);
    Arm arm = new Arm(gyro);
    WheeledGrabber intake = new WheeledGrabber();
    // Controllers
    CommandXboxController driver = new CommandXboxController(0);
    CommandXboxController copilot = new CommandXboxController(1);
    ButtonPanel controlPanel = new ButtonPanel(2);

    public final LogManager logManager = new LogManager(pdh);


    // Commands

    public RobotContainer() {
        pdh.clearStickyFaults();
        ph.clearStickyFaults();
        if (ENABLE_PNEUMATICS) {
            ph.enableCompressorDigital();
        } else {
            ph.disableCompressor();
        }
        initSmartDashboard();
        initAutos();
        configureButtonBindings();
        DriverStation.silenceJoystickConnectionWarning(true);
        LimelightManager.getInstance().addLimelight("limelight-front");
        LimelightManager.getInstance().addLimelight("limelight-rear");
        AutoDrive.postPIDs();
    }

   private void configureButtonBindings() {
        // Driver

        // coop:button(A, Zero Field Oriented [Press],pilot)
        driver.a().onTrue(new InstantCommand(drivetrain::zeroFieldOrientation).andThen(drivetrain::resetAllToAbsolute));
       // coop:button(Y, Zero to current Rotation [Press],pilot)
        driver.y().onTrue(new InstantCommand(drivetrain::zeroFieldOrientationManual).andThen(drivetrain::resetAllToAbsolute));

       // coop:button(LBumper, Substation Left [HOLD],pilot)
        driver.leftBumper().whileTrue(AutoSubstationAlign.get(drivetrain, arm, intake, driver, -Constants.Auto.hpOffsetY));
       // coop:button(RBumper, Substation Right [HOLD],pilot)
        driver.rightBumper().whileTrue(AutoSubstationAlign.get(drivetrain, arm, intake, driver, Constants.Auto.hpOffsetY));
        // Copilot

        controlPanel.onButton(ButtonPanel.PanelButton.STYLE_PURPLE).onTrue(blinkins.commandSet(BlinkinPair.ColorPair.CUBE));
        controlPanel.onButton(ButtonPanel.PanelButton.STYLE_YELLOW).onTrue(blinkins.commandSet(BlinkinPair.ColorPair.CONE));

       //coop:button(LTrigger, Confirm alignment [PRESS], pilot)
        controlPanel.onButton(ButtonPanel.PanelButton.TOP_LEFT     ).whileTrue(new AutoGridScore(drivetrain, arm,Constants.Auto.highCone.withPolePosition(PolePosition.LEFT),    intake, driver));
        controlPanel.onButton(ButtonPanel.PanelButton.TOP_CENTER   ).whileTrue(new AutoGridScore(drivetrain, arm,Constants.Auto.highCube.withPolePosition(PolePosition.CENTER),    intake, driver));
        controlPanel.onButton(ButtonPanel.PanelButton.TOP_RIGHT    ).whileTrue(new AutoGridScore(drivetrain, arm,Constants.Auto.highCone.withPolePosition(PolePosition.RIGHT),    intake, driver));
        controlPanel.onButton(ButtonPanel.PanelButton.MIDDLE_LEFT  ).whileTrue(new AutoGridScore(drivetrain, arm,Constants.Auto.midCone.withPolePosition(PolePosition.LEFT),     intake, driver));
        controlPanel.onButton(ButtonPanel.PanelButton.MIDDLE_CENTER).whileTrue(new AutoGridScore(drivetrain, arm,Constants.Auto.midCube.withPolePosition(PolePosition.CENTER),     intake, driver));
        controlPanel.onButton(ButtonPanel.PanelButton.MIDDLE_RIGHT ).whileTrue(new AutoGridScore(drivetrain, arm,Constants.Auto.midCone.withPolePosition(PolePosition.RIGHT),     intake, driver));
        controlPanel.onButton(ButtonPanel.PanelButton.BOTTOM_LEFT  ).whileTrue(new AutoGridScore(drivetrain, arm,Constants.Auto.hybridNode.withPolePosition(PolePosition.LEFT),  intake, driver));
        controlPanel.onButton(ButtonPanel.PanelButton.BOTTOM_CENTER).whileTrue(new AutoGridScore(drivetrain, arm,Constants.Auto.middleHybridNode.withPolePosition(PolePosition.CENTER),  intake, driver));
        controlPanel.onButton(ButtonPanel.PanelButton.BOTTOM_RIGHT ).whileTrue(new AutoGridScore(drivetrain, arm,Constants.Auto.hybridNode.withPolePosition(PolePosition.RIGHT),  intake, driver));

        // coop:button(A, Run Intake [PRESS],copilot)
        copilot.a().toggleOnTrue(new GrabberIntakeCommand(intake));
        // coop:button(B,Run Outtake [HOLD],copilot)
        copilot.b().whileTrue(new GrabberOuttakeCommand(intake));



        //coop:button(RBumper, Floor pickup [HOLD], copilot)
        copilot.rightBumper().whileTrue(Commands.sequence(
                new RetractAndPivotCommand(arm, Constants.Auto.armDown),
                new SetArmPosition(arm, Constants.Auto.armDown),
                new InstantCommand(new GrabberIntakeCommand(intake)::schedule)));
        //coop:button(LBumper, Set arm upright [HOLD], copilot)
        copilot.leftBumper().whileTrue(new ResetArmPositionCommand(arm));


        // INSPECTION CODE
//        copilot.y().whileTrue(Commands.sequence(
//                new RetractAndPivotCommand(arm, Constants.Auto.highCone.score.getRotation2d()),
//                new ExtensionCommand(arm, Constants.Auto.highCone.score)
//        ));


        //coop:button(X, Downed Cone Intake [ HOLD, copilot)
        copilot.x().whileTrue(Commands.sequence(
            new RetractAndPivotCommand(arm, Constants.Auto.armDownBackwards).withTimeout(1),
            new SetArmPosition(arm, Constants.Auto.armDownBackwards).withTimeout(1),
            new InstantCommand(new GrabberIntakeCommand(intake)::schedule)
        ));

        copilot.y().onTrue(new InstantCommand(arm::resetAngle));

        new Trigger(LimelightManager.getInstance()::canSeeTargets)
                .onTrue(blinkins.commandSet(RevBlinkin.ColorPattern.WAVES_LAVA))
                .onFalse(frontBlinken.commandSetPattern(RevBlinkin.ColorPattern.WAVES_PARTY))
                .onFalse(rearBlinken.commandSetPattern(RevBlinkin.ColorPattern.WAVES_FOREST))
        ;

        AtomicReference<NeutralMode> currentMode = new AtomicReference<>(NeutralMode.Brake);
        new Trigger(RobotController::getUserButton).onTrue(new InstantCommand(()->{
            if (currentMode.get() == NeutralMode.Brake) {
                currentMode.set(NeutralMode.Coast);
            } else {
                currentMode.set(NeutralMode.Brake);
            }
            DataLogManager.log("FPGA User Button: Setting pivot falcons to NeutralMode."+currentMode);
            arm.setRotationNeutralMode(currentMode.get());
        }).ignoringDisable(true));

        // SmartDashboard Commands

    }

    public void setTeleopDefaultCommands() {
        // coop:button(LJoystick, Translate swerve,pilot)
        // coop:button(RJoystick, Rotate swerve [LEFTRIGHT],pilot)
        // coop:button(X, Slow drive [PRESS])
        // coop:button(B, Fast Drive [PRESS])
        drivetrain.setDefaultCommand(new SwerveDriveCommand(drivetrain, driver.getHID()));
        // coop:button(LJoystick, Adjust arm angle [UPDOWN],copilot)
        // coop:button(LTrigger, Extend telescope [HOLD],copilot)
        // coop:button(RTrigger, Retract telescope [HOLD],copilot)
        arm.setDefaultCommand(new ManualArm(arm, copilot));
        intake.setDefaultCommand(new DefaultGrabberCommand(intake));
    }

    public void setAutoDefaultCommands() {
        CommandScheduler.getInstance().removeDefaultCommand(drivetrain);
//        CommandScheduler.getInstance().removeDefaultCommand(arm);
//        CommandScheduler.getInstance().removeDefaultCommand(intake);

    }

    private void initSmartDashboard() {
        SmartDashboard.putNumber("arm/targetX", 22);
        SmartDashboard.putNumber("arm/targetY", 0);
        SmartDashboard.putNumber("arm/targetAngle", 0);
        SmartDashboard.putNumber("arm/targetExtension", 22);
    }

    private void initAutos() {
        AutoManager manager = AutoManager.getInstance();
        manager.addAuto(new Auto1PieceTaxi(drivetrain, arm, intake, ScoringGridLocation.TOP_GRID));
//        manager.addAuto(new Auto1PieceTaxi(drivetrain, arm, intake, ScoringGridLocation.BOTTOM_GRID));
        manager.addAuto(new Auto1PieceBalance(drivetrain, arm, intake,ScoringGridLocation.TOP_GRID));
//        manager.addAuto(new Auto1PieceBalance(drivetrain, arm, intake, ScoringGridLocation.BOTTOM_GRID));
        manager.addAuto(new Auto1PieceBalance(drivetrain, arm, intake, ScoringGridLocation.MIDDLE_GRID));
        manager.addAuto(new Auto2PieceTaxi(drivetrain, arm, intake, ScoringGridLocation.TOP_GRID));
//        manager.addAuto(new Auto2PieceTaxi(drivetrain, arm, intake, ScoringGridLocation.BOTTOM_GRID));
        manager.addAuto("MiddleGrid1PieceSideBalance", new Auto1PieceSideBalance(drivetrain, arm, intake));
        manager.addAuto("MiddleGridSideBalance", new AutoSideBalance(drivetrain, arm, intake));
        manager.addAuto("ScoreHighCube", new AutoGridScore(drivetrain, arm, Constants.Auto.highCube, intake));
        manager.addAuto("ScoreMidCube", new AutoGridScore(drivetrain, arm, Constants.Auto.midCube, intake));
        manager.addDefaultAuto("DoNothing", new InstantCommand(), null);
    }

    public Command getAutonomousCommand() {
        return AutoManager.getInstance().getSelected();
    }

}
