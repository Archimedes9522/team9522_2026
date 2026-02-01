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
  
  /** NavX gyroscope for heading - connected via MXP SPI port */
  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
  
  /** Container for all subsystems, commands, and button bindings */
  private RobotContainer m_robotContainer;
  
  /** Field visualization widget for SmartDashboard/AdvantageScope */
  private final Field2d m_field = new Field2d();
  
  /** Tracks selected auto name to update path preview when changed */
  private String autoName, newAutoName;
  
  /** Power Distribution Hub for voltage/current monitoring */
  private final PowerDistribution m_pdh = new PowerDistribution(1, ModuleType.kRev);
  
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
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
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
    Logger.recordOutput("FieldSimulation/RobotPose", m_robotContainer.getRobotPose());
    Logger.recordOutput("FieldSimulation/AimDirection", m_robotContainer.getAimDirection());
    Logger.recordOutput("FieldSimulation/AimTarget", 
        new Pose3d(m_robotContainer.getAimPoint(), Rotation3d.kZero));
    
    // Update dashboard displays
    updateSmartDashboard();
    
    // Reset thread priority
    Threads.setCurrentThreadPriority(false, 10);
  }

  /** Called once when autonomous mode starts */
  @Override
  public void autonomousInit() {
    // Update aim point based on current alliance
    m_robotContainer.getSuperstructure().updateAimPointForAlliance();
    
    // Get the selected auto from the dashboard chooser
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // Schedule the autonomous command to run
    if (m_autonomousCommand != null) {
      System.out.println("[Auto] Running auto: " + m_autonomousCommand.getName());
      m_autonomousCommand.schedule();
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
    // Cleanup when leaving auto mode - nothing needed currently
  }

  /** Called once when teleop mode starts */
  @Override
  public void teleopInit() {
    // Update aim point based on current alliance
    m_robotContainer.getSuperstructure().updateAimPointForAlliance();
    
    // Cancel autonomous command when teleop starts
    // This ensures driver has full control
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
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
    // === FIELD VISUALIZATION ===
    SmartDashboard.putData("Field", m_field);           // Field visualization with robot pose
    
    // === SYSTEM STATUS ===
    SmartDashboard.putData("PDH", m_pdh);               // Power distribution widget
    SmartDashboard.putData("Scheduler", CommandScheduler.getInstance()); // Running commands
    
    // === ROBOT POSE ===
    SmartDashboard.putNumber("Robot X (m)", 0.0);
    SmartDashboard.putNumber("Robot Y (m)", 0.0);
    SmartDashboard.putNumber("Robot Heading (deg)", 0.0);
    
    // === ROBOT VELOCITY ===
    SmartDashboard.putNumber("Robot Speed (m/s)", 0.0);
    SmartDashboard.putNumber("Robot Vx (m/s)", 0.0);
    SmartDashboard.putNumber("Robot Vy (m/s)", 0.0);
    SmartDashboard.putNumber("Robot Omega (deg/s)", 0.0);
    
    // === GYRO DATA ===
    SmartDashboard.putNumber("Gyro Angle (deg)", m_gyro.getAngle());
    SmartDashboard.putNumber("Gyro Pitch (deg)", m_gyro.getPitch());
    SmartDashboard.putNumber("Gyro Roll (deg)", m_gyro.getRoll());
    
    // === VISION STATUS ===
    SmartDashboard.putNumber("Vision Tags Seen", 0);
    SmartDashboard.putBoolean("Vision Connected", false);
    
    // === MECHANISM STATUS ===
    SmartDashboard.putNumber("Turret Angle (deg)", 0.0);
    SmartDashboard.putNumber("Shooter RPM", 0.0);
    SmartDashboard.putBoolean("Shooter Ready", false);
    SmartDashboard.putBoolean("Auto-Aim Active", false);
    
    // === CONTROL BUTTONS ===
    SmartDashboard.putBoolean("Reset Pose", false);     // Button to reset odometry
    SmartDashboard.putBoolean("Lock Wheels", false);    // Button to lock wheels in X
    SmartDashboard.putBoolean("Zero Gyro", false);      // Button to zero gyro heading
  }

  /** Update SmartDashboard values every loop */
  private void updateSmartDashboard() {
    // === MATCH INFO ===
    SmartDashboard.putNumber("Match Time", Timer.getMatchTime());
    SmartDashboard.putNumber("PDH Voltage", m_pdh.getVoltage());
    SmartDashboard.putNumber("PDH Current (A)", m_pdh.getTotalCurrent());

    // === ROBOT POSE ===
    var robotPose = m_robotContainer.m_robotDrive.getPose();
    m_field.setRobotPose(robotPose);
    SmartDashboard.putNumber("Robot X (m)", robotPose.getX());
    SmartDashboard.putNumber("Robot Y (m)", robotPose.getY());
    SmartDashboard.putNumber("Robot Heading (deg)", robotPose.getRotation().getDegrees());
    
    // === ROBOT VELOCITY ===
    var chassisSpeeds = m_robotContainer.m_robotDrive.getRobotRelativeSpeeds();
    double robotSpeed = Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Robot Speed (m/s)", robotSpeed);
    SmartDashboard.putNumber("Robot Vx (m/s)", chassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Robot Vy (m/s)", chassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Robot Omega (deg/s)", Math.toDegrees(chassisSpeeds.omegaRadiansPerSecond));
    
    // === GYRO DATA ===
    SmartDashboard.putNumber("Gyro Angle (deg)", m_gyro.getAngle());
    SmartDashboard.putNumber("Gyro Pitch (deg)", m_gyro.getPitch());
    SmartDashboard.putNumber("Gyro Roll (deg)", m_gyro.getRoll());
    
    // === MECHANISM STATUS ===
    var superstructure = m_robotContainer.getSuperstructure();
    SmartDashboard.putNumber("Turret Angle (deg)", 
        superstructure.getTurretAngle().in(edu.wpi.first.units.Units.Degrees));
    SmartDashboard.putNumber("Shooter RPM", 
        superstructure.getShooterSpeed().in(edu.wpi.first.units.Units.RPM));
    SmartDashboard.putBoolean("Shooter Ready", superstructure.isReadyToShoot());
    
    // === AIM POINT (for debugging) ===
    var aimPoint = m_robotContainer.getAimPoint();
    SmartDashboard.putNumber("Aim X (m)", aimPoint.getX());
    SmartDashboard.putNumber("Aim Y (m)", aimPoint.getY());
    SmartDashboard.putNumber("Aim Z (m)", aimPoint.getZ());

    // === HANDLE DASHBOARD BUTTONS ===
    // Reset Pose button
    if (SmartDashboard.getBoolean("Reset Pose", false)) {
      m_robotContainer.m_robotDrive.resetOdometry(new Pose2d());
      SmartDashboard.putBoolean("Reset Pose", false);
    }

    // Lock Wheels button
    if (SmartDashboard.getBoolean("Lock Wheels", false)) {
      m_robotContainer.m_robotDrive.lock();
      SmartDashboard.putBoolean("Lock Wheels", false);
    }
    
    // Zero Gyro button
    if (SmartDashboard.getBoolean("Zero Gyro", false)) {
      m_robotContainer.m_robotDrive.zeroGyro();
      SmartDashboard.putBoolean("Zero Gyro", false);
    }
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

    // Add the swerve drivetrain to the physics simulation
    // YAGSL's SwerveDriveSimulation wraps the MapleSim drivetrain
    var swerveDrive = m_robotContainer.m_robotDrive.getSwerveDrive();
    var mapleSimDriveOpt = swerveDrive.getMapleSimDrive();
    if (mapleSimDriveOpt.isPresent()) {
      // YAGSL handles drivetrain registration internally when using maple-sim
      // The SwerveDriveSimulation is YAGSL's wrapper, not MapleSim's type directly
      System.out.println("MapleSim: YAGSL SwerveDriveSimulation available");
      System.out.println("MapleSim: Drivetrain type: " + mapleSimDriveOpt.get().getClass().getName());
    } else {
      System.out.println("MapleSim: WARNING - No drivetrain simulation available. Running in simulation may have limited physics.");
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
