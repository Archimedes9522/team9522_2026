// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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
import frc.robot.commands.ShootOnTheMoveCommand;
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
 * RobotContainer is where we define all subsystems, commands, and button bindings.
 * This follows the "declarative" Command-based paradigm where we declare WHAT the robot
 * should do, not HOW (the subsystems handle the how).
 * 
 * <p>Key concepts:
 * <ul>
 *   <li>Subsystems: Hardware abstractions (drive, vision, etc.)</li>
 *   <li>Commands: Actions the robot performs (drive, auto routines, etc.)</li>
 *   <li>Triggers: Button bindings that activate commands</li>
 * </ul>
 * 
 * <p>Controller bindings are handled by the controls package:
 * <ul>
 *   <li>{@link DriverControls} - Driver controller (port 0)</li>
 *   <li>{@link OperatorControls} - Operator controller (port 1)</li>
 * </ul>
 */
public class RobotContainer {
        // ==================== SUBSYSTEMS ====================
        // Subsystems are created here and persist for the entire robot lifetime.
        // They register themselves with the CommandScheduler automatically.
        
        /** YAGSL-based swerve drive subsystem - handles all driving and odometry */
        public final SwerveSubsystem m_robotDrive = new SwerveSubsystem();
        
        // ==================== MECHANISM SUBSYSTEMS ====================
        // Game-specific mechanisms for 2026 "Rebuilt" game
        
        /** Dual flywheel shooter for launching FUEL */
        private final ShooterSubsystem m_shooter = new ShooterSubsystem();
        
        /** Turret for aiming shooter ±90° */
        private final TurretSubsystem m_turret = new TurretSubsystem();
        
        /** Hood for adjusting shot trajectory */
        private final HoodSubsystem m_hood = new HoodSubsystem();
        
        /** Ground intake with pivot arm */
        private final IntakeSubsystem m_intake = new IntakeSubsystem();
        
        /** Ball storage and queuing */
        private final HopperSubsystem m_hopper = new HopperSubsystem();
        
        /** Final feed stage to shooter */
        private final KickerSubsystem m_kicker = new KickerSubsystem();
        
        /** 
         * Superstructure coordinates all mechanisms for unified control.
         * Provides combined commands for shooting, feeding, and aiming.
         */
        private final Superstructure m_superstructure = new Superstructure(
                m_shooter, m_turret, m_hood, m_intake, m_hopper, m_kicker);
        
        /**
         * Vision subsystem for AprilTag pose estimation.
         * Runs automatically via CommandScheduler.periodic() - no explicit calls needed.
         * Sends pose updates to SwerveSubsystem via the addVisionMeasurement callback.
         */
        @SuppressWarnings("unused") // Field appears unused but runs via CommandScheduler
        private final Vision m_vision;
        
        // ==================== ZONE DETECTION ====================
        /** Current alliance for change detection */
        private Alliance currentAlliance = Alliance.Red;
        
        // ==================== AUTO CHOOSER ====================
        /** Dropdown in SmartDashboard/Shuffleboard to select autonomous routine */
        private final SendableChooser<Command> autoChooser;

        /**
         * Constructor - called once when robot code starts.
         * Sets up all subsystems, commands, and button bindings.
         */
        public RobotContainer() {
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
                                // Real robot - Rubik Pi 3 camera on front
                                // The method reference m_robotDrive::addVisionMeasurement connects
                                // vision pose estimates to the drive's pose estimator
                                m_vision = new Vision(
                                        m_robotDrive::addVisionMeasurement,
                                        new VisionIOPhotonVision(camera0Name, robotToCamera0),
                                        new VisionIO() {});  // Second camera disabled - add later if needed
                                break;

                        case SIM:
                                // Simulation - use simulated PhotonVision cameras
                                // These generate fake AprilTag detections based on simulated robot pose
                                // Mimics CA26's Limelight MegaTag2 behavior using PhotonVision multi-tag
                                m_vision = new Vision(
                                        m_robotDrive::addVisionMeasurement,
                                        new VisionIOPhotonVisionSim(
                                                camera0Name, 
                                                robotToCamera0,
                                                m_robotDrive::getPose),
                                        new VisionIO() {});  // Second camera disabled
                                break;

                        default:
                                // Replay mode - empty IO, data comes from log file
                                m_vision = new Vision(
                                        m_robotDrive::addVisionMeasurement,
                                        new VisionIO() {},
                                        new VisionIO() {});
                                break;
                }

                // ==================== PATHPLANNER NAMED COMMANDS ====================
                // Named commands can be triggered from PathPlanner autonomous paths.
                // In PathPlanner, add an "Event Marker" and use these names.
                NamedCommands.registerCommand("setX", m_robotDrive.setXCommand());
                NamedCommands.registerCommand("zeroHeading", m_robotDrive.zeroHeadingCommand());
                
                // Register mechanism commands for autonomous paths
                NamedCommands.registerCommand("shoot", m_superstructure.shootCommand());
                NamedCommands.registerCommand("intake", m_superstructure.setIntakeDeployAndRoll());
                NamedCommands.registerCommand("feedAll", m_superstructure.feedAllCommand());
                NamedCommands.registerCommand("stopShooting", m_superstructure.stopShootingCommand());
                NamedCommands.registerCommand("stopAll", m_superstructure.stopAllCommand());
                NamedCommands.registerCommand("eject", m_superstructure.ejectCommand());
                
                // Shoot-on-the-move: Enable/disable auto-aim with lead compensation
                // Use "enableAutoAim" at start of shooting section, "disableAutoAim" after
                NamedCommands.registerCommand("enableAutoAim", 
                        new ShootOnTheMoveCommand(m_robotDrive, m_superstructure, 
                                () -> m_superstructure.getAimPoint())
                        .withName("AutoAim.enable"));
                NamedCommands.registerCommand("disableAutoAim", 
                        Commands.runOnce(() -> {})  // Placeholder - auto-aim ends when path ends
                        .withName("AutoAim.disable"));
                
                // Simulation-only: Fire a FUEL projectile for visualization
                // This creates a physics-simulated projectile that shows trajectory in AdvantageScope
                if (Robot.isSimulation()) {
                        NamedCommands.registerCommand("fireFuel", 
                                DriverControls.fireFuel(m_robotDrive, m_superstructure));
                        
                        // Fire continuously while the command is active (10 shots/sec)
                        NamedCommands.registerCommand("fireFuelRepeating",
                                Commands.repeatingSequence(
                                        DriverControls.fireFuel(m_robotDrive, m_superstructure),
                                        Commands.waitSeconds(0.1))
                                .withName("AutoFire.repeating"));
                }
                
                // ==================== CONTROLLER BINDINGS ====================
                // Controller bindings are configured in separate classes for organization.
                // This keeps RobotContainer clean and makes bindings easier to find/modify.
                DriverControls.configure(
                        ControllerConstants.kDriverControllerPort, 
                        m_robotDrive,
                        m_superstructure);  // Pass Superstructure for simulation fireFuel
                OperatorControls.configure(
                        ControllerConstants.kOperatorControllerPort, 
                        m_robotDrive, 
                        m_superstructure);
                PoseControls.configure(
                        ControllerConstants.kPoseControllerPort,
                        m_robotDrive);
                
                // ==================== ZONE-BASED AUTO-AIM ====================
                // Initialize alliance and set up triggers for automatic aim point switching
                // based on where the robot is on the field.
                
                // Initialize alliance (default to red if not present)
                onAllianceChanged(getAlliance());
                
                // Set up trigger to detect alliance changes (e.g., when connected to FMS)
                new Trigger(() -> getAlliance() != currentAlliance)
                        .onTrue(Commands.runOnce(() -> onAllianceChanged(getAlliance()))
                                .ignoringDisable(true));
                
                // Triggers for auto aim point changes based on field position
                // When robot crosses zone boundaries, aim point updates automatically
                new Trigger(() -> isInAllianceZone())
                        .onChange(Commands.runOnce(() -> onZoneChanged())
                                .ignoringDisable(true));
                
                new Trigger(() -> isOnAllianceOutpostSide())
                        .onChange(Commands.runOnce(() -> onZoneChanged())
                                .ignoringDisable(true));
                
                // Silence joystick warnings in simulation
                if (!Robot.isReal()) {
                        DriverStation.silenceJoystickConnectionWarning(true);
                }
                
                // ==================== AUTO CHOOSER ====================
                // AutoBuilder.buildAutoChooser() automatically finds all .auto files
                // in src/main/deploy/pathplanner/autos/ and creates a dropdown
                autoChooser = AutoBuilder.buildAutoChooser();
                
                // Add a "Do Nothing" default option
                autoChooser.setDefaultOption("Do Nothing", Commands.none().withName("Do Nothing"));
                
                // Add any custom autos here
                // autoChooser.addOption("Drive Forward", m_robotDrive.driveForward().withTimeout(5));
                
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
         * This combines the robot pose with the shooter's turret/hood angles.
         * Use this for AdvantageScope 3D visualization.
         * 
         * @return Pose3d representing where the shooter is pointing in field space
         */
        public Pose3d getAimDirection() {
                Pose3d shooterPose = m_superstructure.getShooterPose();
                return m_robotDrive.getPose3d().plus(
                        new Transform3d(shooterPose.getTranslation(), shooterPose.getRotation()));
        }
        
        /**
         * Gets the current aim point (target location).
         * @return Translation3d of the target
         */
        public Translation3d getAimPoint() {
                return m_superstructure.getAimPoint();
        }
        
        /**
         * Sets the aim point for the superstructure.
         * @param aimPoint The target location
         */
        public void setAimPoint(Translation3d aimPoint) {
                m_superstructure.setAimPoint(aimPoint);
        }
        
        /**
         * Gets the superstructure subsystem.
         * @return The Superstructure
         */
        public Superstructure getSuperstructure() {
                return m_superstructure;
        }
        
        // ==================== ZONE DETECTION ====================
        
        /**
         * Gets the current alliance.
         * @return Current alliance, defaults to Red if not connected to FMS
         */
        private Alliance getAlliance() {
                return DriverStation.getAlliance().orElse(Alliance.Red);
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
                if (isInAllianceZone()) {
                        m_superstructure.setAimPoint(Constants.AimPoints.getAllianceHubPosition());
                } else {
                        if (isOnAllianceOutpostSide()) {
                                m_superstructure.setAimPoint(Constants.AimPoints.getAllianceOutpostPosition());
                        } else {
                                m_superstructure.setAimPoint(Constants.AimPoints.getAllianceFarSidePosition());
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
                currentAlliance = alliance;
                
                // Update aim point based on alliance
                if (alliance == Alliance.Blue) {
                        m_superstructure.setAimPoint(Constants.AimPoints.BLUE_HUB.value);
                } else {
                        m_superstructure.setAimPoint(Constants.AimPoints.RED_HUB.value);
                }
                
                // In simulation, reset pose to the correct alliance side
                // BUT only when disabled - don't override auto starting position!
                if (!Robot.isReal() && DriverStation.isDisabled()) {
                        Pose2d newPose = (alliance == Alliance.Blue)
                                ? new Pose2d(2.75, 4.0, Rotation2d.fromDegrees(0))      // Blue: left side
                                : new Pose2d(14.25, 4.0, Rotation2d.fromDegrees(180));  // Red: right side
                        m_robotDrive.resetOdometry(newPose);
                        System.out.println("Reset pose for " + alliance + " alliance: " + newPose);
                }
                
                System.out.println("Alliance changed to: " + alliance);
        }
}
