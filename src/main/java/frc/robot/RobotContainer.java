// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.autos.AutoCommands;
import frc.robot.controls.DriverControls;
import frc.robot.controls.OperatorControls;
import frc.robot.controls.PoseControls;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.mechanisms.HoodSubsystem;
import frc.robot.subsystems.mechanisms.HopperSubsystem;
import frc.robot.subsystems.mechanisms.IntakeSubsystem;
import frc.robot.subsystems.mechanisms.KickerSubsystem;
import frc.robot.subsystems.mechanisms.ShooterSubsystem;
import frc.robot.subsystems.mechanisms.Superstructure;
import frc.robot.subsystems.mechanisms.TurretSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

/**
 * RobotContainer manages all subsystems, commands, and controller bindings.
 */
public class RobotContainer {
        
        /** YAGSL-based swerve drive subsystem - handles all driving and odometry */
        public final SwerveSubsystem m_robotDrive = new SwerveSubsystem();
        
        // ==================== MECHANISM SUBSYSTEMS ====================
        // Game-specific mechanisms for 2026 "Rebuilt" game.
        
        /** Dual flywheel shooter for launching FUEL */
        private final ShooterSubsystem m_shooter;
        
        /** Turret for aiming shooter ±90° */
        private final TurretSubsystem m_turret;
        
        /** Hood for adjusting shot trajectory */
        private final HoodSubsystem m_hood;
        
        /** Ground intake with pivot arm */
        private final IntakeSubsystem m_intake;
        
        /** Ball storage and queuing */
        private final HopperSubsystem m_hopper;
        
        /** Final feed stage to shooter */
        private final KickerSubsystem m_kicker;
        
        /** 
         * Superstructure coordinates all mechanisms for unified control.
         * Provides combined commands for shooting, feeding, and aiming.
         * Null when running in chassis-only mode.
         */
        private final Superstructure m_superstructure;
        
        /**
         * Vision subsystem for AprilTag pose estimation.
         * Runs automatically via CommandScheduler.periodic() - no explicit calls needed.
         * Sends pose updates to SwerveSubsystem via the addVisionMeasurement callback.
         */
        @SuppressWarnings("unused") // Field appears unused but runs via CommandScheduler
        private final Vision m_vision;
        
        // ==================== ZONE DETECTION ====================
        /** Current alliance for change detection (defaults to Blue per WPILib convention) */
        private Alliance currentAlliance = Alliance.Blue;
        
        // ==================== AUTO CHOOSER ====================
        /** Dropdown in SmartDashboard/Shuffleboard to select autonomous routine */
        private final SendableChooser<Command> autoChooser;

        /**
         * Constructor - called once when robot code starts.
         * Sets up all subsystems, commands, and button bindings.
         */
        public RobotContainer() {
                // ==================== MECHANISM SUBSYSTEM INITIALIZATION ====================
                // Only create mechanism subsystems when they are physically connected.
                // In chassis-only mode (kChassisOnly = true), skip initialization to avoid
                // CAN bus timeouts on missing SparkMax/SparkFlex controllers (IDs 15-23).
                // Each missing controller causes ~30s of blocking timeout = 5+ minute startup!
                if (!Constants.kChassisOnly) {
                        m_shooter = new ShooterSubsystem();
                        m_turret = new TurretSubsystem();
                        m_hood = new HoodSubsystem();
                        m_intake = new IntakeSubsystem();
                        m_hopper = new HopperSubsystem();
                        m_kicker = new KickerSubsystem();
                        m_superstructure = new Superstructure(
                                m_shooter, m_turret, m_hood, m_intake, m_hopper, m_kicker);
                } else {
                        System.out.println("*** CHASSIS-ONLY MODE ***");
                        System.out.println("*** Mechanism subsystems DISABLED to avoid CAN timeouts ***");
                        System.out.println("*** Set Constants.kChassisOnly = false when mechanisms are wired ***");
                        m_shooter = null;
                        m_turret = null;
                        m_hood = null;
                        m_intake = null;
                        m_hopper = null;
                        m_kicker = null;
                        m_superstructure = null;
                }

                // ==================== VISION SETUP ====================
                // Vision uses the IO pattern for AdvantageKit logging and replay support.
                // Different IO implementations are used based on robot mode:
                // - REAL: Actual PhotonVision cameras
                // - SIM: Simulated cameras (or dummy for now)
                // - REPLAY: Dummy IO for log replay analysis
                //
                // Currently using SINGLE CAMERA setup (Rubik Pi 3 on front).
                // To add a second camera later, replace the empty VisionIO() with:
                //   new VisionIOPhotonVision(camera1Name, robotToCamera1)
                switch (Constants.currentMode) {
                        case REAL:
                                m_vision = new Vision(
                                        m_robotDrive::addVisionMeasurement,
                                        m_robotDrive::getRotation,
                                        new VisionIOPhotonVision(camera0Name, robotToCamera0),
                                        new VisionIO() {});
                                break;

                        case SIM:
                                m_vision = new Vision(
                                        m_robotDrive::addVisionMeasurement,
                                        m_robotDrive::getRotation,
                                        new VisionIOPhotonVisionSim(
                                                camera0Name,
                                                robotToCamera0,
                                                m_robotDrive::getPose),
                                        new VisionIO() {});
                                break;

                        default:
                                m_vision = new Vision(
                                        m_robotDrive::addVisionMeasurement,
                                        m_robotDrive::getRotation,
                                        new VisionIO() {},
                                        new VisionIO() {});
                                break;
                }

                // ==================== PATHPLANNER NAMED COMMANDS ====================
                // Named commands can be triggered from PathPlanner autonomous paths.
                // All commands are centralized in AutoCommands.java for easy reference.
                AutoCommands.registerAll(m_robotDrive, m_superstructure);
                
                // ==================== CONTROLLER BINDINGS ====================
                DriverControls.configure(
                        ControllerConstants.kDriverControllerPort, 
                        m_robotDrive,
                        m_superstructure);  // null-safe: sim features disabled when chassis-only
                
                // Only configure operator controls when mechanisms exist
                if (m_superstructure != null) {
                        OperatorControls.configure(
                                ControllerConstants.kOperatorControllerPort, 
                                m_robotDrive, 
                                m_superstructure);
                }
                
                PoseControls.configure(
                        ControllerConstants.kPoseControllerPort,
                        m_robotDrive);
                
                // ==================== ZONE-BASED AUTO-AIM ====================
                // Only set up zone detection when mechanisms exist
                if (m_superstructure != null) {
                        // Initialize alliance and set up triggers for automatic aim point switching
                        onAllianceChanged(getAlliance());
                        
                        new Trigger(() -> getAlliance() != currentAlliance)
                                .onTrue(Commands.runOnce(() -> onAllianceChanged(getAlliance()))
                                        .ignoringDisable(true));
                        
                        new Trigger(() -> isInAllianceZone())
                                .onChange(Commands.runOnce(() -> onZoneChanged())
                                        .ignoringDisable(true));
                        
                        new Trigger(() -> isOnAllianceOutpostSide())
                                .onChange(Commands.runOnce(() -> onZoneChanged())
                                        .ignoringDisable(true));
                }
                
                // Silence joystick connection warnings (controller 2 is optional pose controller)
                DriverStation.silenceJoystickConnectionWarning(true);
                
                // ==================== AUTO CHOOSER ====================
                // AutoBuilder.buildAutoChooser() automatically finds all .auto files
                // in src/main/deploy/pathplanner/autos/ and creates a dropdown
                autoChooser = AutoBuilder.buildAutoChooser();
                
                // Add a "Do Nothing" default option
                autoChooser.setDefaultOption("Do Nothing", Commands.none().withName("Do Nothing"));

                // SysId routines — select one, deploy, enable in Test mode, then analyze the log
                // Steps: 1) Select routine  2) Deploy  3) Test mode  4) Enable  5) Run full sequence
                // 6) Disable  7) Download .wpilog from RIO  8) Open in WPILib SysId Tool
                if (m_turret != null) {
                        autoChooser.addOption("[Calibration] Turret Vernier", m_turret.calibrateVernierCommand());
                        autoChooser.addOption("[SysId] Turret", m_turret.sysId());
                }
                if (m_intake != null) {
                        autoChooser.addOption("[SysId] Intake Pivot", m_intake.sysId());
                }
                
                SmartDashboard.putData("Auto Chooser", autoChooser);
        }

        /**
         * Returns the autonomous command selected in the dashboard.
         * Called by Robot.java at the start of autonomous period.
         */
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
        
        // ==================== GETTERS FOR LOGGING ====================
        
        /**
         * Gets the robot's current pose.
         * @return Pose2d of the robot
         */
        public Pose2d getRobotPose() {
                return m_robotDrive.getPose();
        }
        
        /**
         * Gets the robot's current 3D pose.
         * @return Pose3d of the robot
         */
        public Pose3d getRobotPose3d() {
                return m_robotDrive.getPose3d();
        }
        
        /**
         * Gets the aim direction - the shooter pose in field coordinates.
         * Returns robot pose when in chassis-only mode.
         * 
         * @return Pose3d representing where the shooter is pointing in field space
         */
        public Pose3d getAimDirection() {
                if (m_superstructure == null) {
                        return m_robotDrive.getPose3d();
                }
                Pose3d shooterPose = m_superstructure.getShooterPose();
                return m_robotDrive.getPose3d().plus(
                        new Transform3d(shooterPose.getTranslation(), shooterPose.getRotation()));
        }
        
        /**
         * Gets the current aim point (target location).
         * Returns origin when in chassis-only mode.
         * @return Translation3d of the target
         */
        public Translation3d getAimPoint() {
                if (m_superstructure == null) {
                        return new Translation3d();
                }
                return m_superstructure.getAimPoint();
        }
        
        /**
         * Sets the aim point for the superstructure.
         * No-op in chassis-only mode.
         * @param aimPoint The target location
         */
        public void setAimPoint(Translation3d aimPoint) {
                if (m_superstructure != null) {
                        m_superstructure.setAimPoint(aimPoint);
                }
        }
        
        /**
         * Gets the superstructure subsystem.
         * @return The Superstructure, or null in chassis-only mode
         */
        public Superstructure getSuperstructure() {
                return m_superstructure;
        }
        
        /**
         * Resets the robot pose to the alliance-specific starting position.
         * Only works in simulation - does nothing on real robot.
         * Call this in teleopInit when NOT coming from auto.
         */
        public void resetSimPoseForAlliance() {
                if (!Robot.isReal()) {
                        Pose2d startPose = getStartingPoseForAlliance();
                        m_robotDrive.resetOdometry(startPose);
                        System.out.println("[Sim] Reset pose for teleop: " + startPose);
                }
        }
        
        /**
         * Gets the default starting pose based on current alliance.
         * @return Pose2d for the starting position
         */
        public Pose2d getStartingPoseForAlliance() {
                Alliance alliance = getAlliance();
                return (alliance == Alliance.Blue)
                        ? new Pose2d(2.75, 4.0, Rotation2d.fromDegrees(0))      // Blue: left side
                        : new Pose2d(14.25, 4.0, Rotation2d.fromDegrees(180));  // Red: right side
        }
        
        // ==================== ZONE DETECTION ====================
        
        /**
         * Gets the current alliance.
         * Defaults to Blue (WPILib convention — field origin is at the Blue alliance wall).
         * @return Current alliance, defaults to Blue if not connected to FMS
         */
        private Alliance getAlliance() {
                return DriverStation.getAlliance().orElse(Alliance.Blue);
        }
        
        /**
         * Checks if the robot is in its alliance zone (near its own driver station).
         * 
         * <p>Alliance zones:
         * <ul>
         *   <li>Blue: X less than 182 inches (near blue driver station at X=0)</li>
         *   <li>Red: X greater than 469 inches (near red driver station at X=16.5m)</li>
         * </ul>
         * 
         * @return true if robot is in its alliance zone
         */
        private boolean isInAllianceZone() {
                Alliance alliance = getAlliance();
                Distance blueZone = Inches.of(182);
                Distance redZone = Inches.of(469);
                
                if (alliance == Alliance.Blue && m_robotDrive.getPose().getMeasureX().lt(blueZone)) {
                        return true;
                } else if (alliance == Alliance.Red && m_robotDrive.getPose().getMeasureX().gt(redZone)) {
                        return true;
                }
                
                return false;
        }
        
        /**
         * Checks if the robot is on the alliance's outpost side of the field.
         * 
         * <p>The field is divided in half along the Y axis:
         * <ul>
         *   <li>Blue outpost side: Y less than midline (158.84 inches)</li>
         *   <li>Red outpost side: Y greater than midline</li>
         * </ul>
         * 
         * @return true if robot is on the outpost side for its alliance
         */
        private boolean isOnAllianceOutpostSide() {
                Alliance alliance = getAlliance();
                Distance midLine = Inches.of(158.84375);
                
                if (alliance == Alliance.Blue && m_robotDrive.getPose().getMeasureY().lt(midLine)) {
                        return true;
                } else if (alliance == Alliance.Red && m_robotDrive.getPose().getMeasureY().gt(midLine)) {
                        return true;
                }
                
                return false;
        }
        
        /**
         * Called when the robot crosses a zone boundary.
         * Updates the aim point based on current field position.
         * 
         * <p>Logic:
         * <ul>
         *   <li>In alliance zone → aim at alliance hub</li>
         *   <li>Not in zone, on outpost side → aim at outpost</li>
         *   <li>Not in zone, on far side → aim at far side target</li>
         * </ul>
         */
        private void onZoneChanged() {
                if (m_superstructure == null) return;
                
                if (isInAllianceZone()) {
                        m_superstructure.setAimPoint(FieldConstants.AimPoints.getAllianceHubPosition());
                } else {
                        if (isOnAllianceOutpostSide()) {
                                m_superstructure.setAimPoint(FieldConstants.AimPoints.getAllianceOutpostPosition());
                        } else {
                                m_superstructure.setAimPoint(FieldConstants.AimPoints.getAllianceFarSidePosition());
                        }
                }
        }
        
        /**
         * Called when alliance color changes (e.g., when connected to FMS).
         * Updates the current alliance and resets aim point to hub.
         * In simulation, also resets the robot pose to the correct side of the field
         * (but only when disabled, not during auto/teleop).
         * 
         * @param alliance The new alliance color
         */
        private void onAllianceChanged(Alliance alliance) {
                System.out.println("[Alliance] Change detected: " + currentAlliance + " -> " + alliance);
                currentAlliance = alliance;
                
                // Update aim point based on alliance (only if mechanisms exist)
                if (m_superstructure != null) {
                        if (alliance == Alliance.Blue) {
                                m_superstructure.setAimPoint(FieldConstants.AimPoints.BLUE_HUB.value);
                        } else {
                                m_superstructure.setAimPoint(FieldConstants.AimPoints.RED_HUB.value);
                        }
                }
                
                // Reset heading when alliance changes (only when disabled to avoid disrupting matches)
                // This ensures field-oriented driving uses the correct heading for the alliance.
                // On real robots: only resets heading (keeps X, Y from current pose)
                // In simulation: resets full pose to the correct alliance side
                if (DriverStation.isDisabled()) {
                        if (!Robot.isReal()) {
                                Pose2d newPose = (alliance == Alliance.Blue)
                                        ? new Pose2d(2.75, 4.0, Rotation2d.fromDegrees(0))
                                        : new Pose2d(14.25, 4.0, Rotation2d.fromDegrees(180));
                                m_robotDrive.resetOdometry(newPose);
                                System.out.println("[Alliance] Sim: Reset pose to: " + newPose);
                        } else {
                                m_robotDrive.resetHeadingForAlliance(alliance);
                                System.out.println("[Alliance] Real: Reset heading for " + alliance);
                        }
                } else {
                        System.out.println("[Alliance] Skipping heading reset (robot enabled)");
                }
        }
}
