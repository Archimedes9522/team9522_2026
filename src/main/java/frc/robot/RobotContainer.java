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
import frc.robot.subsystems.mechanisms.LEDSubsystem;
import frc.robot.subsystems.mechanisms.ShooterSubsystem;
import frc.robot.subsystems.mechanisms.Superstructure;
import frc.robot.subsystems.mechanisms.TurretSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

/** Manages all subsystems, commands, and controller bindings. */
public class RobotContainer {

	public final SwerveSubsystem m_robotDrive = new SwerveSubsystem();

	// Mechanism subsystems (null in chassis-only mode)
	private final ShooterSubsystem m_shooter;
	private final TurretSubsystem m_turret;
	private final HoodSubsystem m_hood;
	private final IntakeSubsystem m_intake;
	private final HopperSubsystem m_hopper;
	private final KickerSubsystem m_kicker;
	private final LEDSubsystem m_led;
	private final Superstructure m_superstructure;

	@SuppressWarnings("unused") // Runs via CommandScheduler
	private final Vision m_vision;

	private Alliance currentAlliance = Alliance.Blue;
	private final SendableChooser<Command> autoChooser;

	public RobotContainer() {
		// Mechanism initialization (skip in chassis-only to avoid CAN timeouts)
		if (!Constants.kChassisOnly) {
			m_shooter = new ShooterSubsystem();
			m_turret = new TurretSubsystem();
			m_hood = new HoodSubsystem();
			m_intake = new IntakeSubsystem();
			m_hopper = new HopperSubsystem();
			m_kicker = new KickerSubsystem();
			m_led = new LEDSubsystem();
			m_superstructure = new Superstructure(
					m_shooter, m_turret, m_hood, m_intake, m_hopper, m_kicker, m_led);
		} else {
			System.out.println("*** CHASSIS-ONLY MODE — mechanisms disabled ***");
			m_shooter = null;
			m_turret = null;
			m_hood = null;
			m_intake = null;
			m_hopper = null;
			m_kicker = null;
			m_led = null;
			m_superstructure = null;
		}

		// Vision (IO pattern for AdvantageKit replay support)
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
						new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, m_robotDrive::getPose),
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

		// PathPlanner named commands
		AutoCommands.registerAll(m_robotDrive, m_superstructure);

		// Controller bindings
		DriverControls.configure(ControllerConstants.kDriverControllerPort, m_robotDrive, m_superstructure);
		if (m_superstructure != null) {
			OperatorControls.configure(ControllerConstants.kOperatorControllerPort, m_robotDrive, m_superstructure);
		}
		PoseControls.configure(ControllerConstants.kPoseControllerPort, m_robotDrive);

		// Zone-based auto-aim & pre-spin
		if (m_superstructure != null) {
			onAllianceChanged(getAlliance());
			new Trigger(() -> getAlliance() != currentAlliance)
					.onTrue(Commands.runOnce(() -> onAllianceChanged(getAlliance())).ignoringDisable(true));
			new Trigger(this::isInAllianceZone)
					.onChange(Commands.runOnce(this::onZoneChanged).ignoringDisable(true));
			new Trigger(this::isOnAllianceOutpostSide)
					.onChange(Commands.runOnce(this::onZoneChanged).ignoringDisable(true));

			// Pre-Spin toggle: active when NOT in our own defensive zone (meaning we are crossing field to shoot)
			new Trigger(this::isInAllianceZone).negate()
					.onChange(Commands.runOnce(() -> m_superstructure.setPreSpinActive(!isInAllianceZone()))
					.ignoringDisable(true));
		}

		DriverStation.silenceJoystickConnectionWarning(true);

		// Auto chooser
		autoChooser = AutoBuilder.buildAutoChooser();
		autoChooser.setDefaultOption("Do Nothing", Commands.none().withName("Do Nothing"));
		autoChooser.addOption("[SysId] Swerve Drive Motors", m_robotDrive.sysIdDriveMotors());
		autoChooser.addOption("[SysId] Swerve Angle Motors", m_robotDrive.sysIdAngleMotors());
		if (m_turret != null) {
			autoChooser.addOption("[Calibration] Turret Vernier", m_turret.calibrateVernierCommand());
			autoChooser.addOption("[SysId] Turret", m_turret.sysId());
		}
		if (m_intake != null) {
			autoChooser.addOption("[SysId] Intake Pivot", m_intake.sysId());
		}
		SmartDashboard.putData("Auto Chooser", autoChooser);
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}

	// ==================== Getters ====================

	public Pose2d getRobotPose() { return m_robotDrive.getPose(); }
	public Pose3d getRobotPose3d() { return m_robotDrive.getPose3d(); }

	public Pose3d getAimDirection() {
		if (m_superstructure == null) return m_robotDrive.getPose3d();
		Pose3d shooterPose = m_superstructure.getShooterPose();
		return m_robotDrive.getPose3d().plus(
				new Transform3d(shooterPose.getTranslation(), shooterPose.getRotation()));
	}

	public Translation3d getAimPoint() {
		return m_superstructure != null ? m_superstructure.getAimPoint() : new Translation3d();
	}

	public void setAimPoint(Translation3d aimPoint) {
		if (m_superstructure != null) m_superstructure.setAimPoint(aimPoint);
	}

	public Superstructure getSuperstructure() { return m_superstructure; }

	public void resetSimPoseForAlliance() {
		if (!Robot.isReal()) {
			Pose2d startPose = getStartingPoseForAlliance();
			m_robotDrive.resetOdometry(startPose);
		}
	}

	public Pose2d getStartingPoseForAlliance() {
		Alliance alliance = getAlliance();
		return (alliance == Alliance.Blue)
				? new Pose2d(2.75, 4.0, Rotation2d.fromDegrees(0))
				: new Pose2d(14.25, 4.0, Rotation2d.fromDegrees(180));
	}

	// ==================== Zone Detection ====================

	private Alliance getAlliance() {
		return DriverStation.getAlliance().orElse(Alliance.Blue);
	}

	/** Robot is in its alliance zone (near own driver station). */
	private boolean isInAllianceZone() {
		Alliance alliance = getAlliance();
		Distance blueZone = Inches.of(182);
		Distance redZone = Inches.of(469);
		if (alliance == Alliance.Blue && m_robotDrive.getPose().getMeasureX().lt(blueZone)) return true;
		if (alliance == Alliance.Red && m_robotDrive.getPose().getMeasureX().gt(redZone)) return true;
		return false;
	}

	/** Robot is on the alliance's outpost side (field divided along Y). */
	private boolean isOnAllianceOutpostSide() {
		Alliance alliance = getAlliance();
		Distance midLine = Inches.of(158.84375);
		if (alliance == Alliance.Blue && m_robotDrive.getPose().getMeasureY().lt(midLine)) return true;
		if (alliance == Alliance.Red && m_robotDrive.getPose().getMeasureY().gt(midLine)) return true;
		return false;
	}

	/** Updates aim point based on robot's field position. */
	private void onZoneChanged() {
		if (m_superstructure == null) return;
		if (isInAllianceZone()) {
			m_superstructure.setAimPoint(FieldConstants.AimPoints.getAllianceHubPosition());
		} else if (isOnAllianceOutpostSide()) {
			m_superstructure.setAimPoint(FieldConstants.AimPoints.getAllianceOutpostPosition());
		} else {
			m_superstructure.setAimPoint(FieldConstants.AimPoints.getAllianceFarSidePosition());
		}
	}

	/** Updates alliance state, resets aim point and heading. */
	private void onAllianceChanged(Alliance alliance) {
		System.out.println("[Alliance] Changed: " + currentAlliance + " -> " + alliance);
		currentAlliance = alliance;

		if (m_superstructure != null) {
			m_superstructure.setAimPoint(
					alliance == Alliance.Blue
							? FieldConstants.AimPoints.BLUE_HUB.value
							: FieldConstants.AimPoints.RED_HUB.value);
		}

		if (DriverStation.isDisabled()) {
			if (!Robot.isReal()) {
				Pose2d newPose = (alliance == Alliance.Blue)
						? new Pose2d(2.75, 4.0, Rotation2d.fromDegrees(0))
						: new Pose2d(14.25, 4.0, Rotation2d.fromDegrees(180));
				m_robotDrive.resetOdometry(newPose);
			} else {
				m_robotDrive.resetHeadingForAlliance(alliance);
			}
		}
	}
}
