package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import org.ironmaple.simulation.SimulatedArena;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import frc.robot.util.CommandsLogging;
import frc.robot.util.maplesim.Arena2026Rebuilt;

/** Main Robot class. Extends LoggedRobot for AdvantageKit logging and replay. */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private boolean comingFromAuto = false;

  @SuppressWarnings("unused") // Used indirectly by YAGSL via swervedrive.json
  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

  private RobotContainer m_robotContainer;
  private final Field2d m_field = new Field2d();
  private String autoName, newAutoName;
  private final PowerDistribution m_pdh = new PowerDistribution(1, ModuleType.kRev);

  // Alerts
  private final Alert chassisOnlyAlert =
    new Alert("Chassis-only mode enabled (mechanisms disabled)", AlertType.kWarning);
  private final Alert lowBatteryAlert =
    new Alert("Battery voltage low (< 11.5V)", AlertType.kWarning);
  private final Alert visionDisconnectedAlert =
    new Alert("Vision camera disconnected", AlertType.kWarning);
  private final Alert visionNoTagsAlert =
    new Alert("Vision connected but no tags visible", AlertType.kInfo);
  private final Alert shooterNotReadyAlert =
    new Alert("Shooter not at speed", AlertType.kInfo);
  private final Alert autoAimInactiveAlert =
    new Alert("Auto-aim inactive", AlertType.kInfo);

  private SimulatedArena m_arena;

  public Robot() {
    // AdvantageKit metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0: Logger.recordMetadata("GitDirty", "All changes committed"); break;
      case 1: Logger.recordMetadata("GitDirty", "Uncommitted changes"); break;
      default: Logger.recordMetadata("GitDirty", "Unknown"); break;
    }

    // AdvantageKit data receivers
    switch (Constants.currentMode) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case SIM:
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case REPLAY:
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    Logger.start();
    CommandsLogging.initialize();
    m_robotContainer = new RobotContainer();
    setupSmartDashboard();
    PortForwarder.add(5800, "photonvision.local", 5800);
  }

  @Override
  public void robotPeriodic() {
    Threads.setCurrentThreadPriority(true, 99);
    CommandScheduler.getInstance().run();
    CommandsLogging.logCommands();

    Logger.recordOutput("FieldSimulation/AimDirection", m_robotContainer.getAimDirection());
    if (m_robotContainer.getSuperstructure() != null) {
      Logger.recordOutput("FieldSimulation/AimTarget",
          new Pose3d(m_robotContainer.getAimPoint(), Rotation3d.kZero));
    }

    updateSmartDashboard();
    Threads.setCurrentThreadPriority(false, 10);
  }

  @Override
  public void autonomousInit() {
    comingFromAuto = true;
    // Reset odometry to set starting pose — alliance is known by now from FMS.
    // Robot must be placed facing downfield (away from driver station).
    m_robotContainer.m_robotDrive.resetOdometry(m_robotContainer.getStartingPoseForAlliance());
    if (m_robotContainer.getSuperstructure() != null) {
      m_robotContainer.getSuperstructure().updateAimPointForAlliance();
    }
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      System.out.println("[Auto] Running: " + m_autonomousCommand.getName());
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void teleopInit() {
    if (m_robotContainer.getSuperstructure() != null) {
      m_robotContainer.getSuperstructure().updateAimPointForAlliance();
    }
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // Vision disabled — reset heading at teleop start since alliance is now known from FMS.
    // Only reset if NOT coming from auto (auto already set the pose, and the robot may
    // have moved during auto so we don't want to overwrite the current position).
    if (!comingFromAuto) {
      m_robotContainer.m_robotDrive.resetOdometry(m_robotContainer.getStartingPoseForAlliance());
    }
    comingFromAuto = false;
  }

  @Override public void teleopPeriodic() {}
  @Override public void teleopExit() {}

  public void setupSmartDashboard() {
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putData("PDH", m_pdh);
    SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());
    SmartDashboard.putBoolean("Chassis Only", Constants.kChassisOnly);
  }

  private void updateSmartDashboard() {
    var robotPose = m_robotContainer.m_robotDrive.getPose();
    m_field.setRobotPose(robotPose);

    SmartDashboard.putNumber("Elastic/MatchTime", Timer.getMatchTime());
    SmartDashboard.putNumber("Elastic/Robot/X", robotPose.getX());
    SmartDashboard.putNumber("Elastic/Robot/Y", robotPose.getY());
    SmartDashboard.putNumber("Elastic/Robot/HeadingDeg", robotPose.getRotation().getDegrees());
    var chassisSpeeds = m_robotContainer.m_robotDrive.getRobotRelativeSpeeds();
    SmartDashboard.putNumber("Elastic/Robot/SpeedMps",
        Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond));
    SmartDashboard.putNumber("Elastic/Power/Voltage", m_pdh.getVoltage());

    var superstructure = m_robotContainer.getSuperstructure();
    if (superstructure != null) {
      SmartDashboard.putNumber("Elastic/Turret/AngleDeg",
          superstructure.getTurretAngle().in(edu.wpi.first.units.Units.Degrees));
      SmartDashboard.putNumber("Elastic/Shooter/RPM",
          superstructure.getShooterSpeed().in(edu.wpi.first.units.Units.RPM));
      SmartDashboard.putBoolean("Elastic/Shooter/Ready", superstructure.isReadyToShoot());
      SmartDashboard.putBoolean("Elastic/Intake/Deployed", superstructure.intake.isDeployed());
    }

    // Alerts
    chassisOnlyAlert.set(Constants.kChassisOnly);
    lowBatteryAlert.set(m_pdh.getVoltage() < 11.5);
    boolean visionConnected = SmartDashboard.getBoolean("Vision Connected", false);
    visionDisconnectedAlert.set(!visionConnected);
    visionNoTagsAlert.set(visionConnected && SmartDashboard.getNumber("Vision Tags Seen", 0) == 0);
    if (superstructure != null) {
      shooterNotReadyAlert.set(!superstructure.isReadyToShoot());
      Command currentCmd = superstructure.getCurrentCommand();
      autoAimInactiveAlert.set(currentCmd == null || !currentCmd.getName().contains("ShootOnTheMove"));
    } else {
      shooterNotReadyAlert.set(false);
      autoAimInactiveAlert.set(false);
    }
  }

  @Override
  public void disabledPeriodic() {
    Command selectedAuto = m_robotContainer.getAutonomousCommand();
    if (selectedAuto == null) return;

    newAutoName = selectedAuto.getName();
    if (!newAutoName.equals(autoName)) {
      autoName = newAutoName;
      if (AutoBuilder.getAllAutoNames().contains(autoName)) {
        System.out.println("Displaying " + autoName);
        try {
          List<PathPlannerPath> pathPlannerPaths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
          List<Pose2d> poses = new ArrayList<>();
          for (PathPlannerPath path : pathPlannerPaths) {
            poses.addAll(
                path.getAllPathPoints().stream()
                    .map(point -> new Pose2d(
                            point.position.getX(),
                            point.position.getY(),
                            new Rotation2d()))
                    .collect(Collectors.toList()));
          }
          m_field.getObject("path").setPoses(poses);
        } catch (IOException | ParseException e) {
          e.printStackTrace();
        }
      } else {
        m_field.getObject("path").setPoses(new ArrayList<>());
      }
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override public void testPeriodic() {}

  // ==================== Simulation ====================

  @Override
  public void simulationInit() {
    SimulatedArena.getInstance().shutDown();

    // addRampCollider: false = only hub is solid (can drive on ramps)
    SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));
    m_arena = SimulatedArena.getInstance();

    // YAGSL manages drivetrain physics in its own relocated MapleSim instance.
    // Arena2026Rebuilt handles game pieces/field obstacles in a separate physics world.
    var swerveDrive = m_robotContainer.m_robotDrive.getSwerveDrive();
    var mapleSimDriveOpt = swerveDrive.getMapleSimDrive();
    if (mapleSimDriveOpt.isPresent()) {
      System.out.println("MapleSim: YAGSL drivetrain simulation active");
    } else {
      System.out.println("MapleSim: WARNING - No drivetrain simulation available");
    }
  }

  @Override
  public void simulationPeriodic() {
    if (m_arena != null) {
      m_arena.simulationPeriodic();
      Pose3d[] fuelPoses = m_arena.getGamePiecesArrayByType("Fuel");
      if (fuelPoses != null && fuelPoses.length > 0) {
        Logger.recordOutput("FieldSimulation/FuelPoses", fuelPoses);
      }
    }
  }
}
