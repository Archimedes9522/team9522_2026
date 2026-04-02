// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.TurretConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.MechanismPositionConfig.Plane;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

/**
 * Turret subsystem using YAMS Pivot for position control and EasyCRT for
 * absolute position via 19t/21t Vernier encoders on the 200t turret ring.
 *
 * <p>Hardware:
 * <ul>
 *   <li>1x NEO motor (SparkMax)</li>
 *   <li>40:1 gear reduction (4:1 REV Sport Gearbox x 200:20 gear transmission)</li>
 *   <li>2x REV Through Bore Encoders (19t and 21t gears on 200t ring, CRT for absolute position)</li>
 *   <li>Non-continuous rotation (limited travel)</li>
 * </ul>
 *
 * <p>At startup, the EasyCRT solver computes the absolute turret angle from the
 * two Vernier encoders and seeds the motor encoder. During operation, the motor
 * encoder tracks normally without continuous CRT interference.
 */
public class TurretSubsystem extends SubsystemBase {

  /** Maximum rotation in one direction (degrees) */
  private static final double MAX_ONE_DIR_FOV = TurretConstants.kMaxAngleDegrees;

  /** Turret position relative to robot center (meters) */
  public static final Translation3d TURRET_TRANSLATION = new Translation3d(-0.205, 0.0, 0.375);

  /** CRT disagreement must exceed this to be considered for a reseed (degrees) */
  private static final double RESEED_THRESHOLD_DEG = 5.0;
  /** Turret velocity must be below this to allow a reseed (degrees/sec) */
  private static final double RESEED_MAX_VELOCITY_DEG_PER_SEC = 5.0;
  /** CRT reading must be stable for this many consecutive cycles before reseeding */
  private static final int RESEED_STABLE_CYCLES = 5;

  // === HARDWARE ===
  private final SparkMax spark;

  // === YAMS CONTROLLER ===
  private final SmartMotorController motorController;
  private final Pivot turret;

  // === ABSOLUTE ENCODERS (VERNIER) ===
  private final DutyCycleEncoder encoderA; // 19t gear, DIO 0
  private final DutyCycleEncoder encoderB; // 21t gear, DIO 1
  private final EasyCRT crtSolver;

  // === CRT RESEED STATE ===
  private double lastCrtDeg = Double.NaN;
  private int crtStableCycles = 0;

  /**
   * Creates a new TurretSubsystem.
   */
  public TurretSubsystem() {
    // Initialize motor (NEO uses SparkMax)
    spark = new SparkMax(TurretConstants.kMotorId, MotorType.kBrushless);

    // Explicitly set feedback sensor to the internal motor encoder.
    // If a Through Bore Encoder is plugged into the data port, the SparkMax
    // may default to using it for PID feedback — which causes oscillation
    // because of the 10:1 gearing mismatch.
    SparkMaxConfig sparkConfig = new SparkMaxConfig();
    sparkConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    // DO NOT set encoder.inverted() for primary encoder in brushless mode — it's determined by motor inversion
    spark.configure(sparkConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    // Initialize Vernier Encoders
    encoderA = new DutyCycleEncoder(TurretConstants.kEncoderAChannel);
    encoderB = new DutyCycleEncoder(TurretConstants.kEncoderBChannel);

    // Configure YAMS EasyCRT for absolute position from 19t/21t Vernier encoders
    // Encoders mesh directly with the 200t turret ring (commonRatio = 1.0)
    EasyCRTConfig crtConfig = new EasyCRTConfig(
            () -> Rotations.of(encoderA.get()),
            () -> Rotations.of(encoderB.get()))
        .withCommonDriveGear(1.0, 200, 19, 21)
        .withAbsoluteEncoderOffsets(
            Rotations.of(-TurretConstants.kEncoderAOffset),
            Rotations.of(-TurretConstants.kEncoderBOffset))
        .withMechanismRange(
            Degrees.of(-MAX_ONE_DIR_FOV - 10),
            Degrees.of(MAX_ONE_DIR_FOV + 10))
        .withAbsoluteEncoderInversions(false, false);

    crtSolver = new EasyCRT(crtConfig);

    // Configure YAMS SmartMotorController
    SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(
            TurretConstants.kP,
            TurretConstants.kI,
            TurretConstants.kD,
            DegreesPerSecond.of(TurretConstants.kMaxVelocityDegPerSec),
            DegreesPerSecondPerSecond.of(TurretConstants.kMaxAccelDegPerSecSq))
        .withFeedforward(new SimpleMotorFeedforward(TurretConstants.kS, TurretConstants.kV, TurretConstants.kA))
        .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(4, 10)))  // 40:1 total
        .withMotorInverted(true)
        .withIdleMode(MotorMode.COAST)
        .withSoftLimit(Degrees.of(-MAX_ONE_DIR_FOV), Degrees.of(MAX_ONE_DIR_FOV))
        .withStatorCurrentLimit(Amps.of(TurretConstants.kCurrentLimitAmps))
        .withClosedLoopRampRate(Seconds.of(0.02))
        .withOpenLoopRampRate(Seconds.of(0.05));

    motorController = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);

    // Starting position — seed from CRT if encoders are connected, otherwise 0°
    Angle startingPosition = Degrees.of(0);
    if (encoderA.isConnected() && encoderB.isConnected()) {
      Optional<Angle> crtAngle = crtSolver.getAngleOptional();
      if (crtAngle.isPresent()) {
        startingPosition = crtAngle.get();
        System.out.println("[Turret] CRT startup seed: " + startingPosition.in(Degrees) + "°"
            + " (status=" + crtSolver.getLastStatus() + ")");
      } else {
        System.out.println("[Turret] CRT solve failed at startup (status=" + crtSolver.getLastStatus()
            + "), defaulting to 0°");
      }
    } else {
      System.out.println("[Turret] Vernier encoders not connected, defaulting to 0°");
    }

    PivotConfig turretConfig = new PivotConfig(motorController)
        .withHardLimit(Degrees.of(-MAX_ONE_DIR_FOV - 5), Degrees.of(MAX_ONE_DIR_FOV + 5))
        .withStartingPosition(startingPosition)
        .withMOI(KilogramSquareMeters.of(0.05))
        .withTelemetry("Turret", TelemetryVerbosity.HIGH)
        .withMechanismPositionConfig(
            new MechanismPositionConfig()
                .withMovementPlane(Plane.XY)
                .withRelativePosition(TURRET_TRANSLATION));

    turret = new Pivot(turretConfig);
  }

  // ==================== COMMANDS ====================

  /**
   * Directly sets the turret target angle. Call this every loop when doing
   * dynamic aiming from a command that already requires this subsystem.
   *
   * @param angle Target angle (0 = turret forward, positive = left, negative = right)
   *              Note: turret forward points toward the robot rear due to backwards mounting.
   */
  public void setTargetAngle(Angle angle) {
    double angleDeg = angle.in(Degrees);
    double clampedDeg = Math.max(-MAX_ONE_DIR_FOV, Math.min(MAX_ONE_DIR_FOV, angleDeg));
    motorController.setPosition(Degrees.of(clampedDeg));
  }

  /**
   * Sets the turret to a specific angle.
   *
   * @param angle Target angle (0 = forward, positive = clockwise)
   * @return Command that moves to the angle
   */
  public Command setAngle(Angle angle) {
    return turret.setAngle(angle);
  }

  /**
   * Sets the turret to a dynamic angle from a supplier.
   *
   * @param turretAngleSupplier Supplier that provides target angle
   * @return Command that continuously updates angle
   */
  public Command setAngleDynamic(Supplier<Angle> turretAngleSupplier) {
    return turret.setAngle(turretAngleSupplier);
  }

  /**
   * Centers the turret (returns to 0 degrees).
   *
   * @return Command that centers the turret
   */
  public Command center() {
    return turret.setAngle(Degrees.of(0));
  }

  /**
   * Sets the turret to open-loop duty cycle control.
   *
   * @param dutyCycle Motor output (-1 to 1)
   * @return Command that applies the duty cycle
   */
  public Command set(double dutyCycle) {
    return turret.set(dutyCycle);
  }

  /**
   * Reseeds the motor encoder from the CRT absolute position.
   * Falls back to zeroing if the CRT can't solve.
   *
   * @return Command that reseeds the encoder
   */
  public Command rezero() {
    return Commands.runOnce(() -> {
      if (encoderA.isConnected() && encoderB.isConnected()) {
        Optional<Angle> crtAngle = crtSolver.getAngleOptional();
        if (crtAngle.isPresent()) {
          motorController.setEncoderPosition(crtAngle.get());
          System.out.println("[Turret] Rezeroed from CRT: " + crtAngle.get().in(Degrees) + "°");
          return;
        }
      }
      // Fallback: manual zero (turret must be physically centered)
      motorController.setEncoderPosition(Degrees.of(0));
      System.out.println("[Turret] Rezeroed to 0° (CRT unavailable)");
    }, this).withName("Turret.Rezero");
  }

  /**
   * Reads raw absolute fractions from Encoder A and B and prints them.
   * Copy the output into Constants.java offset values when the turret is perfectly centered at 0.
   */
  public Command calibrateVernierCommand() {
    return Commands.runOnce(() -> {
      System.out.println("=============== TURRET VERNIER CALIBRATION ===============");
      System.out.println("Set kEncoderAOffset = " + encoderA.get());
      System.out.println("Set kEncoderBOffset = " + encoderB.get());
      System.out.println("==========================================================");
    }).ignoringDisable(true).withName("Turret.CalibrateVernier");
  }

  /**
   * Runs system identification with reduced CAN jitter for better data quality.
   * Bypasses YAMS and logs directly from SparkMax with fast CAN status frames (5ms)
   * to ensure fresh encoder data every robot loop iteration.
   *
   * @return SysId command sequence
   */
  public Command sysId() {
    final double gearRatio = TurretConstants.kGearRatio;
    final double limitRad = Math.toRadians(MAX_ONE_DIR_FOV - 5); // 75° safety margin

    SysIdRoutine routine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(0.5).per(Second), // Quasistatic ramp: 0.5 V/s (slow for more data)
            Volts.of(4),               // Dynamic step: 4V
            Seconds.of(10),            // Timeout
            (state) -> Logger.recordOutput("Turret/SysIdState", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            (voltage) -> spark.setVoltage(voltage.in(Volts)),
            (log) -> {
              double motorPos = spark.getEncoder().getPosition();   // motor rotations
              double motorVel = spark.getEncoder().getVelocity();   // motor RPM
              log.motor("TurretMotor")
                  .voltage(Volts.of(spark.getAppliedOutput() * spark.getBusVoltage()))
                  .angularPosition(Radians.of(motorPos / gearRatio * 2.0 * Math.PI))
                  .angularVelocity(RadiansPerSecond.of(motorVel / gearRatio * 2.0 * Math.PI / 60.0));
            },
            this
        )
    );

    // Grace period prevents immediate stop when starting a test near a limit
    final double[] testStartTime = {0};
    BooleanSupplier atLimit = () -> {
      if (Timer.getFPGATimestamp() - testStartTime[0] < 0.5) return false;
      return Math.abs(turret.getAngle().in(Radians)) >= limitRad;
    };

    return Commands.sequence(
        // Configure fast CAN status frames for consistent SysId data
        Commands.runOnce(() -> {
          SparkMaxConfig sysIdConfig = new SparkMaxConfig();
          sysIdConfig.signals
              .primaryEncoderPositionPeriodMs(5)
              .primaryEncoderVelocityPeriodMs(5)
              .appliedOutputPeriodMs(5);
          spark.configure(sysIdConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }),
        Commands.runOnce(motorController::stopClosedLoopController),
        Commands.print("[Turret SysId] Starting — fast CAN frames enabled"),
        Commands.runOnce(() -> testStartTime[0] = Timer.getFPGATimestamp()),
        routine.quasistatic(SysIdRoutine.Direction.kForward).until(atLimit),
        Commands.runOnce(() -> testStartTime[0] = Timer.getFPGATimestamp()),
        routine.quasistatic(SysIdRoutine.Direction.kReverse).until(atLimit),
        Commands.runOnce(() -> testStartTime[0] = Timer.getFPGATimestamp()),
        routine.dynamic(SysIdRoutine.Direction.kForward).until(atLimit),
        Commands.runOnce(() -> testStartTime[0] = Timer.getFPGATimestamp()),
        routine.dynamic(SysIdRoutine.Direction.kReverse).until(atLimit)
    ).finallyDo(() -> {
      // Restore default CAN status frame periods
      SparkMaxConfig defaultConfig = new SparkMaxConfig();
      defaultConfig.signals
          .primaryEncoderPositionPeriodMs(20)
          .primaryEncoderVelocityPeriodMs(20)
          .appliedOutputPeriodMs(10);
      spark.configure(defaultConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      motorController.startClosedLoopController();
      System.out.println("[Turret SysId] Complete — CAN frames restored");
    }).withName("Turret.SysId");
  }

  // ==================== GETTERS ====================

  /**
   * Gets the turret angle adjusted for robot coordinate frame.
   * Since the turret is mounted backwards, this adds 180 degrees offset.
   *
   * @return Angle in robot frame
   */
  public Angle getRobotAdjustedAngle() {
    return turret.getAngle().plus(Degrees.of(180));
  }

  /**
   * Gets the raw turret angle without adjustment.
   *
   * @return Raw angle from encoder
   */
  public Angle getRawAngle() {
    return turret.getAngle();
  }

  /**
   * Checks if the turret is at the target angle (within tolerance).
   *
   * @param targetDegrees Target angle in degrees
   * @param toleranceDegrees Acceptable error in degrees
   * @return True if at target
   */
  public boolean isAtAngle(double targetDegrees, double toleranceDegrees) {
    double currentDegrees = getRawAngle().in(Degrees);
    return Math.abs(currentDegrees - targetDegrees) < toleranceDegrees;
  }

  // ==================== PERIODIC ====================

  @Override
  public void periodic() {
    turret.updateTelemetry();

    boolean connected = encoderA.isConnected() && encoderB.isConnected();

    if (connected) {
      // Solve CRT for absolute angle
      Optional<Angle> crtAngle = crtSolver.getAngleOptional();
      double crtDeg = crtAngle.map(a -> a.in(Degrees)).orElse(Double.NaN);
      double yamsDeg = turret.getAngle().in(Degrees);

      Logger.recordOutput("Turret/CRT/AngleDeg", crtDeg);
      Logger.recordOutput("Turret/CRT/Status", crtSolver.getLastStatus().name());
      Logger.recordOutput("Turret/CRT/ErrorRot", crtSolver.getLastErrorRotations());
      Logger.recordOutput("Turret/Vernier/RawA", encoderA.get());
      Logger.recordOutput("Turret/Vernier/RawB", encoderB.get());

      // Stationary-only reseed: correct encoder drift without disturbing the PID.
      // Conditions: CRT solved, disagreement > threshold, turret nearly stopped,
      // AND CRT reading has been stable (consistent) for several consecutive cycles.
      // This prevents mid-movement reseeds that cause sudden encoder jumps and oscillation.
      if (crtAngle.isPresent()) {
        double errorDeg = Math.abs(crtDeg - yamsDeg);
        // spark.getEncoder().getVelocity() returns motor RPM (brushless).
        // Convert to mechanism deg/sec: RPM × (360°/rot) / (40:1 gear) / (60 s/min)
        double velocityDegPerSec = Math.abs(spark.getEncoder().getVelocity() * 360.0 / (TurretConstants.kGearRatio * 60.0));
        Logger.recordOutput("Turret/CRT/VsYamsErrorDeg", errorDeg);

        // Check CRT reading stability (consecutive cycles within threshold of each other)
        if (!Double.isNaN(lastCrtDeg) && Math.abs(crtDeg - lastCrtDeg) < RESEED_THRESHOLD_DEG) {
          crtStableCycles++;
        } else {
          crtStableCycles = 0;
        }
        lastCrtDeg = crtDeg;

        boolean shouldReseed = errorDeg > RESEED_THRESHOLD_DEG
            && velocityDegPerSec < RESEED_MAX_VELOCITY_DEG_PER_SEC
            && crtStableCycles >= RESEED_STABLE_CYCLES;

        if (shouldReseed) {
          motorController.setEncoderPosition(crtAngle.get());
          crtStableCycles = 0; // reset after reseeding
          Logger.recordOutput("Turret/CRT/Reseeded", true);
        } else {
          Logger.recordOutput("Turret/CRT/Reseeded", false);
        }
        Logger.recordOutput("Turret/CRT/StableCycles", crtStableCycles);
        Logger.recordOutput("Turret/CRT/VelocityDegPerSec", velocityDegPerSec);
      }
    }

    Logger.recordOutput("Turret/AngleDegrees", getRawAngle().in(Degrees));
    Logger.recordOutput("Turret/VernierConnected", connected);
    Logger.recordOutput("ASCalibration/FinalComponentPoses", new Pose3d[] {
        new Pose3d(
            TURRET_TRANSLATION,
            new Rotation3d(0, 0, turret.getAngle().in(Radians)))
    });
  }

  @Override
  public void simulationPeriodic() {
    turret.simIterate();
  }
}
