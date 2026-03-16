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

/**
 * Main Robot class - the entry point for all robot code.
 * 
 * This extends LoggedRobot (from AdvantageKit) instead of TimedRobot for enhanced logging.
 * AdvantageKit provides:
 * - Deterministic replay: Re-run matches exactly from log files
 * - Automatic logging: All inputs/outputs recorded for debugging
 * - NT4 publishing: Real-time data to dashboard tools
 * 
 * Robot Lifecycle:
 * 1. Constructor: Called once when robot code starts
 * 2. robotPeriodic(): Called every 20ms regardless of mode
 * 3. xxxInit(): Called once when entering a mode (auto, teleop, etc.)
 * 4. xxxPeriodic(): Called every 20ms while in that mode
 * 5. xxxExit(): Called once when leaving a mode
 */
public class Robot extends LoggedRobot {
  /** The currently scheduled autonomous command */
  private Command m_autonomousCommand;
  
  /** Tracks if we're coming from autonomous mode (to know if teleop should reset pose) */
  private boolean comingFromAuto = false;
  
  /** NavX gyroscope for heading - connected via MXP SPI port */
  @SuppressWarnings("unused") // Used indirectly by YAGSL via swervedrive.json config
  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
  
  /** Container for all subsystems, commands, and button bindings */
  private RobotContainer m_robotContainer;
  
  /** Field visualization widget for SmartDashboard/AdvantageScope */
  private final Field2d m_field = new Field2d();
  
  /** Tracks selected auto name to update path preview when changed */
  private String autoName, newAutoName;
  
  /** Power Distribution Hub for voltage/current monitoring */
  private final PowerDistribution m_pdh = new PowerDistribution(1, ModuleType.kRev);

  // === ALERTS (Driver Station / SmartDashboard Alerts) ===
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
  
  /** MapleSim arena for 2026 "Rebuilt" game simulation */
  private SimulatedArena m_arena;

  /**
   * Robot constructor - called once when robot code starts.
   * Sets up AdvantageKit logging and creates RobotContainer.
   */
  public Robot() {
    // ==================== ADVANTAGEKIT METADATA ====================
    // Record build info for debugging - helps identify which code version was deployed
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncommitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // ==================== ADVANTAGEKIT DATA RECEIVERS ====================
    // Configure where log data is sent based on robot mode
    switch (Constants.currentMode) {
      case REAL:
        // Real robot: Log to USB stick AND publish to NetworkTables
        // USB logs saved to /U/logs on the roboRIO
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Simulation: Only publish to NetworkTables (no file logging)
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replay mode: Read from log file and write analysis results
        // Used to re-analyze matches after the fact
        setUseTiming(false); // Run as fast as possible (not real-time)
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start the AdvantageKit logger - must be called after configuring receivers
    Logger.start();

    // Initialize command logging for AdvantageScope visualization
    CommandsLogging.initialize();

    // Create RobotContainer - this initializes all subsystems and bindings
    m_robotContainer = new RobotContainer();
    setupSmartDashboard();
    
    // Port forward PhotonVision web interface through the RoboRIO for debugging
    PortForwarder.add(5800, "photonvision.local", 5800);
  }

  /**
   * Called every 20ms regardless of robot mode.
   * This is the main robot loop where commands are executed.
   */
  @Override
  public void robotPeriodic() {
    // Set high thread priority for consistent loop timing
    Threads.setCurrentThreadPriority(true, 99);
    
    // Run the command scheduler - this executes all active commands
    // and calls periodic() on all subsystems
    CommandScheduler.getInstance().run();
    
    // Log command activity to AdvantageScope
    CommandsLogging.logCommands();
    
    // Log field simulation data for AdvantageScope 3D visualization
    // Note: Robot pose is logged as "Odometry/Robot" in SwerveSubsystem.periodic()
    Logger.recordOutput("FieldSimulation/AimDirection", m_robotContainer.getAimDirection());
    if (m_robotContainer.getSuperstructure() != null) {
      Logger.recordOutput("FieldSimulation/AimTarget", 
          new Pose3d(m_robotContainer.getAimPoint(), Rotation3d.kZero));
    }
    
    // Update dashboard displays
    updateSmartDashboard();
    
    // Reset thread priority
    Threads.setCurrentThreadPriority(false, 10);
  }

  /** Called once when autonomous mode starts */
  @Override
  public void autonomousInit() {
    comingFromAuto = true;
    
    // Update aim point based on current alliance (only if mechanisms exist)
    if (m_robotContainer.getSuperstructure() != null) {
      m_robotContainer.getSuperstructure().updateAimPointForAlliance();
    }
    
    // Get the selected auto from the dashboard chooser
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // Schedule the autonomous command to run
    if (m_autonomousCommand != null) {
      System.out.println("[Auto] Running auto: " + m_autonomousCommand.getName());
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    } else {
      System.out.println("[Auto] No autonomous command selected!");
    }
  }

  @Override
  public void autonomousPeriodic() {
    // Commands run automatically via CommandScheduler - nothing needed here
  }

  @Override
  public void autonomousExit() {
    // comingFromAuto stays true - teleop will check it
  }

  /** Called once when teleop mode starts */
  @Override
  public void teleopInit() {
    // Update aim point based on current alliance (only if mechanisms exist)
    if (m_robotContainer.getSuperstructure() != null) {
      m_robotContainer.getSuperstructure().updateAimPointForAlliance();
    }
    
    // Cancel autonomous command when teleop starts
    // This ensures driver has full control
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
    // In simulation, reset pose to alliance starting position
    // BUT only if we're NOT coming from auto (auto already set the pose)
    if (!isReal() && !comingFromAuto) {
      m_robotContainer.resetSimPoseForAlliance();
    }
    
    // Reset the flag for next time
    comingFromAuto = false;
  }

  @Override
  public void teleopPeriodic() {
    // Commands run automatically via CommandScheduler - nothing needed here
  }

  @Override
  public void teleopExit() {
    // Cleanup when leaving teleop mode - nothing needed currently
  }

  /** Initialize SmartDashboard/Shuffleboard widgets */
  public void setupSmartDashboard() {
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putData("PDH", m_pdh);
    SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());
    SmartDashboard.putBoolean("Chassis Only", Constants.kChassisOnly);
  }

  /** Update SmartDashboard values every loop */
  private void updateSmartDashboard() {
    var robotPose = m_robotContainer.m_robotDrive.getPose();
    m_field.setRobotPose(robotPose);

    // All pose/velocity/module data is logged via AdvantageKit Logger in
    // SwerveSubsystem.periodic() and robotPeriodic(). Only publish what
    // the Elastic driver dashboard actually needs here.
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
  }

  /**
   * Called every 20ms while robot is disabled.
   * Used to preview selected auto path on the field widget.
   */
  @Override
  public void disabledPeriodic() {
    // Check if selected auto has changed
    Command selectedAuto = m_robotContainer.getAutonomousCommand();
    if (selectedAuto == null) {
      return; // No auto selected
    }
    
    newAutoName = selectedAuto.getName();
    if (!newAutoName.equals(autoName)) {
      autoName = newAutoName;
      
      // If this is a valid PathPlanner auto, display its path on the field
      if (AutoBuilder.getAllAutoNames().contains(autoName)) {
        System.out.println("Displaying " + autoName);
        try {
          // Load all paths from the auto file
          List<PathPlannerPath> pathPlannerPaths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
          
          // Convert path points to poses for Field2d display
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
          
          // Display path on Field2d widget
          m_field.getObject("path").setPoses(poses);
        } catch (IOException | ParseException e) {
          e.printStackTrace();
        }
      } else {
        // Clear the path if this isn't a PathPlanner auto
        m_field.getObject("path").setPoses(new ArrayList<>());
      }
    }
  }

  /** Called once when test mode starts */
  @Override
  public void testInit() {
    // Cancel all running commands for clean test environment
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
    // Test mode periodic - nothing needed currently
  }

  // ==================== SIMULATION METHODS ====================
  
  /**
   * Called once when simulation mode starts.
   * Sets up the MapleSim arena for the 2026 "Rebuilt" game.
   */
  @Override
  public void simulationInit() {
    // Shut down any existing arena instance to release physics bodies
    SimulatedArena.getInstance().shutDown();

    // Create and register the 2026 Rebuilt arena
    // Parameter: addRampCollider
    //   true  = Ramps around hubs are solid obstacles (can't drive on them)
    //   false = Only the hub itself is a collider (can drive on ramps)
    // 
    // If you're hitting invisible walls, try setting to false - the ramp 
    // colliders extend 217 inches which might not match AdvantageScope's field
    boolean addRampCollider = false;  
    
    SimulatedArena.overrideInstance(new Arena2026Rebuilt(addRampCollider));
    m_arena = SimulatedArena.getInstance();

    // NOTE: YAGSL bundles its own relocated MapleSim under swervelib.simulation.ironmaple.*,
    // which is a DIFFERENT class hierarchy from org.ironmaple.simulation.* used by Arena2026Rebuilt.
    // YAGSL auto-registers the drivetrain with its own internal SimulatedArena and calls
    // simulationPeriodic() during updateOdometry(), so drivetrain physics (translation, rotation,
    // and gyro simulation) work independently of this arena.
    //
    // The Arena2026Rebuilt handles game pieces, field obstacles, hubs, and outposts in a separate
    // physics world. Game piece-robot interactions (collisions, intake) won't work across the two
    // worlds. To unify them, Arena2026Rebuilt would need to extend YAGSL's SimulatedArena instead
    // of the standalone MapleSim's SimulatedArena.
    var swerveDrive = m_robotContainer.m_robotDrive.getSwerveDrive();
    var mapleSimDriveOpt = swerveDrive.getMapleSimDrive();
    if (mapleSimDriveOpt.isPresent()) {
      System.out.println("MapleSim: YAGSL drivetrain simulation active (managed by YAGSL's internal SimulatedArena)");
    } else {
      System.out.println("MapleSim: WARNING - No drivetrain simulation available.");
    }
  }

  /**
   * Called every 20ms during simulation.
   * Updates the MapleSim physics simulation.
   */
  @Override
  public void simulationPeriodic() {
    if (m_arena != null) {
      m_arena.simulationPeriodic();
      
      // Log game piece positions for AdvantageScope 3D visualization
      Pose3d[] fuelPoses = m_arena.getGamePiecesArrayByType("Fuel");
      if (fuelPoses != null && fuelPoses.length > 0) {
        Logger.recordOutput("FieldSimulation/FuelPoses", fuelPoses);
      }
    }
  }
}
