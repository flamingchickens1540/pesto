package org.team1540.robot2023;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
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
import org.team1540.robot2023.commands.grabber.*;
import org.team1540.robot2023.commands.vision.TurnToGamePiece;
import org.team1540.robot2023.utils.*;

import static org.team1540.robot2023.Constants.ENABLE_PNEUMATICS;

import javax.swing.plaf.metal.OceanTheme;

public class RobotContainer {
    // Hardware

    AHRS gyro = new AHRS(SPI.Port.kMXP);
    BlinkinManager blinkins = BlinkinManager.getInstance();
    RevBlinkin frontBlinken = blinkins.frontBlinkin;
    RevBlinkin rearBlinken = blinkins.rearBlinkin;

    public final PneumaticHub ph = new PneumaticHub(Constants.PNEUMATIC_HUB);
    public final PowerDistribution pdh = new PowerDistribution(Constants.PDH, PowerDistribution.ModuleType.kRev);
    // Subsystems

    Drivetrain drivetrain = new Drivetrain(gyro);
    Arm arm = new Arm();
    WheeledGrabber intake = new WheeledGrabber();

    // Controllers
    CommandXboxController driver = new CommandXboxController(0);
    CommandXboxController copilot = new CommandXboxController(1);
    ButtonPanel controlPanel = new ButtonPanel(2);

    RevBlinkin.ColorPattern frontPattern = BlinkinManager.ColorPair.TELEOP.front;
    boolean armIsBrakeMode = false;

    // Commands



    public RobotContainer() {
        pdh.clearStickyFaults();
        ph.clearStickyFaults();
        setNeutralModes();
        if (ENABLE_PNEUMATICS) {
            ph.enableCompressorDigital();
        } else {
            ph.disableCompressor();
        }
        initSmartDashboard();
        initAutos();
        configureButtonBindings();
        DriverStation.silenceJoystickConnectionWarning(true);
        AutoDrive.postPIDs();
    }

   private void configureButtonBindings() {
        // Driver

        // coop:button(A, Zero Field Oriented [Press],pilot)
//        driver.a().onTrue(new InstantCommand(drivetrain::zeroFieldOrientation).andThen(drivetrain::resetAllToAbsolute).withName("ZeroFieldOrientation"));
       // coop:button(Y, Zero to current Rotation [Press],pilot)
        driver.y().onTrue(new InstantCommand(drivetrain::zeroFieldOrientationManual).andThen(drivetrain::resetAllToAbsolute).withName("ZeroFieldOrientationManual"));
        driver.rightTrigger().whileTrue(new GrabberAggressiveCommand(intake));
       // coop:button(LBumper, Substation Left [HOLD],pilot)
        driver.leftBumper().whileTrue(AutoSubstationAlign.get(drivetrain, arm, intake, driver, -Constants.Auto.hpOffsetY));
       // coop:button(RBumper, Substation Right [HOLD],pilot)
        driver.rightBumper().whileTrue(AutoSubstationAlign.get(drivetrain, arm, intake, driver, Constants.Auto.hpOffsetY));
        //Coop: button(B, Cone vision [HOLD], pilot)
        driver.b().whileTrue(new TurnToGamePiece(drivetrain,driver, TurnToGamePiece.GamePiece.CONE));
        //Coop: button(X, Cube vision [HOLD], pilot)
        driver.x().whileTrue(new TurnToGamePiece(drivetrain,driver, TurnToGamePiece.GamePiece.CUBE));


        // Copilot
//        driver.start().onTrue(new InstantCommand(drivetrain::updateWithApriltags).andThen(new PrintCommand("Rezeroing")).ignoringDisable(true));
        controlPanel.onButton(ButtonPanel.PanelButton.STYLE_PURPLE).onTrue(blinkins.commandSetGamepiece(false));
        controlPanel.onButton(ButtonPanel.PanelButton.STYLE_YELLOW).onTrue(blinkins.commandSetGamepiece(true));

       //coop:button(LTrigger, Confirm alignment [PRESS], pilot)
        controlPanel.onButton(ButtonPanel.PanelButton.TOP_LEFT     ).whileTrue(new AutoCone(drivetrain, arm,Constants.Auto.highCone.withPolePosition(PolePosition.LEFT),    intake, driver, true));
        controlPanel.onButton(ButtonPanel.PanelButton.TOP_CENTER   ).whileTrue(new AutoCube(drivetrain, arm,Constants.Auto.highCube.withPolePosition(PolePosition.CENTER),    intake, true));
        controlPanel.onButton(ButtonPanel.PanelButton.TOP_RIGHT    ).whileTrue(new AutoCone(drivetrain, arm,Constants.Auto.highCone.withPolePosition(PolePosition.RIGHT),    intake, driver, true));
        controlPanel.onButton(ButtonPanel.PanelButton.MIDDLE_LEFT  ).whileTrue(new AutoCone(drivetrain, arm,Constants.Auto.midCone.withPolePosition(PolePosition.LEFT),     intake, driver, true));
        controlPanel.onButton(ButtonPanel.PanelButton.MIDDLE_CENTER).whileTrue(new AutoCube(drivetrain, arm,Constants.Auto.midCube.withPolePosition(PolePosition.CENTER),     intake, true));
        controlPanel.onButton(ButtonPanel.PanelButton.MIDDLE_RIGHT ).whileTrue(new AutoCone(drivetrain, arm,Constants.Auto.midCone.withPolePosition(PolePosition.RIGHT),     intake, driver, true));
        controlPanel.onButton(ButtonPanel.PanelButton.BOTTOM_LEFT  ).whileTrue(new AutoHybrid(drivetrain, arm,Constants.Auto.hybridNode.withPolePosition(PolePosition.LEFT),  intake, driver));
        controlPanel.onButton(ButtonPanel.PanelButton.BOTTOM_CENTER).whileTrue(new AutoHybrid(drivetrain, arm,Constants.Auto.middleHybridNode.withPolePosition(PolePosition.CENTER),  intake, driver));
        controlPanel.onButton(ButtonPanel.PanelButton.BOTTOM_RIGHT ).whileTrue(new AutoHybrid(drivetrain, arm,Constants.Auto.hybridNode.withPolePosition(PolePosition.RIGHT),  intake, driver));

        // coop:button(A, Run Intake [PRESS],copilot)
        copilot.a().toggleOnTrue(new GrabberIntakeCommand(intake));
        // coop:button(B,Run Outtake [HOLD],copilot)
        copilot.b().whileTrue(new GrabberOuttakeCommand(intake));



        //coop:button(RBumper, Floor pickup [HOLD], copilot)
        copilot.rightBumper().whileTrue(Commands.sequence(
                new InstantCommand(new GrabberIntakeCommand(intake)::schedule),
                new SetArmPosition(arm, Constants.Auto.armDown)));
        //coop:button(LBumper, Set arm upright [HOLD], copilot)
       copilot.leftBumper().whileTrue(new ResetArmPositionCommand(arm));


        // INSPECTION CODE
//        copilot.y().whileTrue(Commands.sequence(
//                new RetractAndPivotCommand(arm, Constants.Auto.highCone.score.getRotation2d()),
//                new ExtensionCommand(arm, Constants.Auto.highCone.score)
//        ));


        //coop:button(X, Downed Cone Intake [ HOLD, copilot)
        copilot.x().whileTrue(Commands.sequence(
                new InstantCommand(new GrabberIntakeCommand(intake)::schedule),
                new SetArmPosition(arm, Constants.Auto.armDownBackwards)
        ).withName("DownedConeIntake"));

       copilot.y().whileTrue(new SetArmPosition(arm, ArmState.fromRotationExtension(Rotation2d.fromDegrees(-30), 0)).andThen(new ZeroArmPositionCommand(arm)));


        new Trigger(LimelightManager.getInstance()::canSeeTargets)
                .onTrue(new InstantCommand(() -> {
                    int closestTime= AutoDrive.getClosestTag(drivetrain);
                    if (closestTime == 4 || closestTime == 5) {
                        rearBlinken.setPattern(BlinkinManager.ColorPair.APRILTAG.rear);
                    } else {
                        blinkins.set(BlinkinManager.ColorPair.APRILTAG);
                    }

                }))
                .onFalse(blinkins.commandSet(BlinkinManager.ColorPair.TELEOP));

        new Trigger(RobotController::getUserButton).onTrue(new InstantCommand(() -> {
            armIsBrakeMode = !armIsBrakeMode;
            setNeutralModes();
        }).withName("InstantToggleBreakMode").ignoringDisable(true));

        // SmartDashboard Commands

    }

    public void setNeutralModes() {
        DataLogManager.log("FPGA User Button: Setting pivot falcons to Brake: "+armIsBrakeMode);
        arm.setRotationNeutralMode(armIsBrakeMode ? NeutralMode.Brake : NeutralMode.Coast);
        arm.setExtensionNeutralMode(armIsBrakeMode ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
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
        manager.addAuto(new Auto1PieceTaxi(drivetrain, arm, intake, ScoringGridLocation.BOTTOM_GRID));
        manager.addAuto(new AutoMiddleGrid1PieceBalance(drivetrain, arm, intake));
        manager.addAuto(new AutoMiddleGrid1PieceTaxiBalance(drivetrain, arm, intake));
//        manager.addAuto(new Auto1PieceBalance(drivetrain, arm, intake, ScoringGridLocation.BOTTOM_GRID));
        manager.addAuto(new AutoTopGrid1PieceBalance(drivetrain, arm, intake));
        manager.addAuto(new AutoBottomGrid1PieceBalance(drivetrain, arm, intake));

        manager.addAuto(new Auto2PieceTaxiCone(drivetrain, arm, intake, ScoringGridLocation.TOP_GRID));
        manager.addAuto(new AutoTopGrid2PieceTaxi(drivetrain, arm, intake));
        manager.addAuto(new AutoTopGrid3PieceTaxi(drivetrain, arm, intake));
        manager.addAuto(new AutoBottomGrid2PieceTaxi(drivetrain, arm, intake));
        manager.addAuto(new AutoBottomGrid2_5PieceTaxi(drivetrain, arm, intake));

        manager.addAuto(new AutoTopGrid2PieceVision(drivetrain, arm, intake, LimelightManager.getInstance().rearLimelight));
        manager.addAuto(new AutoBottomGrid2PieceTaxiVision(drivetrain, arm, intake, LimelightManager.getInstance().rearLimelight));
        manager.addAuto(new AutoBottomGrid2_5PieceTaxiVision(drivetrain, arm, intake, LimelightManager.getInstance().rearLimelight));

        //manager.addAuto(new Auto2PieceTaxiConeVision(drivetrain, arm, intake, ScoringGridLocation.TOP_GRID));
//        manager.addAuto(new Auto2PieceTaxi(drivetrain, arm, intake, ScoringGridLocation.BOTTOM_GRID));
//        manager.addAuto("MiddleGrid1PieceSideBalance", new Auto1PieceSideBalance(drivetrain, arm, intake));
//        manager.addAuto("MiddleGridSideBalance", new AutoSideBalance(drivetrain, arm, intake));
        manager.addAuto("ScoreHighCube", new AutoCube(drivetrain, arm, Constants.Auto.highCube.withPolePosition(PolePosition.CENTER), intake, true));
        manager.addAuto("ScoreMidCube", new AutoHybrid(drivetrain, arm, Constants.Auto.highCube.withPolePosition(PolePosition.CENTER), intake));
        manager.addDefaultAuto("DoNothing", new InstantCommand(), null);
    }

    public Command getAutonomousCommand() {
        return AutoManager.getInstance().getSelected();
    }

}
