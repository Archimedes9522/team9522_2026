// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

/**
 * Intake subsystem with pivot arm and rollers.
 * 
 * <p>The intake deploys to the ground to collect FUEL balls,
 * then stows inside the frame perimeter for transport.
 * 
 * <p>Hardware:
 * <ul>
 *   <li>1x NEO motor for pivot arm (5:1 × 5:1 × 60/18 = 83.33:1 reduction)</li>
 *   <li>1x NEO motor for rollers (using SparkMax instead of ThriftyNova)</li>
 * </ul>
 */
public class IntakeSubsystem extends SubsystemBase {

  // === PIVOT POSITIONS (degrees) ===
  private static final double STOW_ANGLE = 0;
  private static final double FEED_ANGLE = 59;
  private static final double HOLD_ANGLE = 115;
  private static final double DEPLOYED_ANGLE = 148;

  // === MOTORS ===
  private final SparkMax pivotMotor;
  private final SparkMax rollerMotor;

  // === YAMS CONTROLLERS ===
  private final SmartMotorController pivotController;
  private final SmartMotorController rollerController;
  private final Arm intakePivot;
  private final FlyWheel intakeRoller;

  /**
   * Creates a new IntakeSubsystem.
   */
  public IntakeSubsystem() {
    // Initialize motors
    pivotMotor = new SparkMax(IntakeConstants.kPivotMotorId, MotorType.kBrushless);
    rollerMotor = new SparkMax(IntakeConstants.kRollerMotorId, MotorType.kBrushless);

    // === ROLLER CONFIGURATION ===
    SmartMotorControllerConfig rollerConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.OPEN_LOOP)
        .withTelemetry("IntakeRollerMotor", TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))  // Direct drive
        .withMotorInverted(true)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(IntakeConstants.kRollerCurrentLimitAmps));

    rollerController = new SparkWrapper(rollerMotor, DCMotor.getNEO(1), rollerConfig);

    FlyWheelConfig rollerFlyWheelConfig = new FlyWheelConfig(rollerController)
        .withDiameter(Inches.of(4))
        .withMass(Pounds.of(0.5))
        .withUpperSoftLimit(RPM.of(6000))
        .withLowerSoftLimit(RPM.of(-6000))
        .withTelemetry("IntakeRoller", TelemetryVerbosity.HIGH);

    intakeRoller = new FlyWheel(rollerFlyWheelConfig);

    // === PIVOT CONFIGURATION ===
    SmartMotorControllerConfig pivotConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(
            25, 0, 0,
            DegreesPerSecond.of(360),
            DegreesPerSecondPerSecond.of(360))
        .withFeedforward(new SimpleMotorFeedforward(0, 10, 0))
        .withTelemetry("IntakePivotMotor", TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 5, 60.0 / 18.0)))
        .withMotorInverted(false)
        .withIdleMode(MotorMode.COAST)
        .withSoftLimit(Degrees.of(0), Degrees.of(150))
        .withStatorCurrentLimit(Amps.of(IntakeConstants.kPivotCurrentLimitAmps))
        .withClosedLoopRampRate(Seconds.of(0.1))
        .withOpenLoopRampRate(Seconds.of(0.1));

    pivotController = new SparkWrapper(pivotMotor, DCMotor.getNEO(1), pivotConfig);

    ArmConfig armConfig = new ArmConfig(pivotController)
        .withSoftLimits(Degrees.of(0), Degrees.of(150))
        .withHardLimit(Degrees.of(0), Degrees.of(155))
        .withStartingPosition(Degrees.of(0))
        .withLength(Feet.of(1))
        .withMass(Pounds.of(2))
        .withTelemetry("IntakePivot", TelemetryVerbosity.HIGH);

    intakePivot = new Arm(armConfig);
  }

  // ==================== ROLLER COMMANDS ====================

  /**
   * Runs the intake rollers to collect FUEL.
   * 
   * @return Command that runs while held
   */
  public Command intakeCommand() {
    return intakeRoller.set(IntakeConstants.kIntakeSpeed)
        .finallyDo(() -> rollerController.setDutyCycle(0))
        .withName("Intake.Run");
  }

  /**
   * Runs the intake rollers in reverse to eject FUEL.
   * 
   * @return Command that runs while held
   */
  public Command ejectCommand() {
    return intakeRoller.set(IntakeConstants.kOuttakeSpeed)
        .finallyDo(() -> rollerController.setDutyCycle(0))
        .withName("Intake.Eject");
  }

  // ==================== PIVOT COMMANDS ====================

  /**
   * Sets the pivot to a specific angle.
   * 
   * @param angle Target angle
   * @return Command that moves to the angle
   */
  public Command setPivotAngle(Angle angle) {
    return intakePivot.setAngle(angle).withName("IntakePivot.SetAngle");
  }

  /**
   * Stows the intake inside the frame perimeter.
   * 
   * @return Command that stows the intake
   */
  public Command stow() {
    return setPivotAngle(Degrees.of(STOW_ANGLE));
  }

  /**
   * Deploys the intake to the ground for collecting.
   * 
   * @return Command that deploys the intake
   */
  public Command deploy() {
    return setPivotAngle(Degrees.of(DEPLOYED_ANGLE));
  }

  /**
   * Positions the intake for feeding balls to the hopper.
   * 
   * @return Command that moves to feed position
   */
  public Command feedPosition() {
    return setPivotAngle(Degrees.of(FEED_ANGLE));
  }

  /**
   * Positions the intake for holding balls.
   * 
   * @return Command that moves to hold position
   */
  public Command holdPosition() {
    return setPivotAngle(Degrees.of(HOLD_ANGLE));
  }

  // ==================== COMBINED COMMANDS ====================

  /**
   * Deploys the intake and runs rollers while held.
   * Stops roller and holds position when released.
   * 
   * @return Command that deploys and runs
   */
  public Command deployAndRollCommand() {
    return Commands.run(() -> {
      pivotController.setPosition(Degrees.of(DEPLOYED_ANGLE));
      rollerController.setDutyCycle(IntakeConstants.kIntakeSpeed);
    }, this).finallyDo(() -> {
      rollerController.setDutyCycle(0);
      pivotController.setPosition(Degrees.of(HOLD_ANGLE));
    }).withName("Intake.DeployAndRoll");
  }

  /**
   * Deploys intake and runs rollers in reverse for unjamming.
   * 
   * @return Command that deploys and reverses
   */
  public Command deployAndEjectCommand() {
    return Commands.run(() -> {
      pivotController.setPosition(Degrees.of(DEPLOYED_ANGLE));
      rollerController.setDutyCycle(IntakeConstants.kOuttakeSpeed);
    }, this).finallyDo(() -> {
      rollerController.setDutyCycle(0);
      pivotController.setPosition(Degrees.of(HOLD_ANGLE));
    }).withName("Intake.DeployAndEject");
  }

  /**
   * Resets the pivot encoder to zero.
   * Use when pivot is physically at the stowed position.
   * 
   * @return Command that resets the encoder
   */
  public Command rezero() {
    return Commands.runOnce(() -> pivotMotor.getEncoder().setPosition(0), this)
        .withName("IntakePivot.Rezero");
  }

  // ==================== GETTERS ====================

  /**
   * Gets the current pivot angle.
   * 
   * @return Current angle
   */
  public Angle getPivotAngle() {
    return intakePivot.getAngle();
  }

  /**
   * Checks if the intake is deployed.
   * 
   * @return True if at or near deployed position
   */
  public boolean isDeployed() {
    return getPivotAngle().in(Degrees) > DEPLOYED_ANGLE - 10;
  }

  /**
   * Checks if the intake is stowed.
   * 
   * @return True if at or near stowed position
   */
  public boolean isStowed() {
    return getPivotAngle().in(Degrees) < STOW_ANGLE + 10;
  }

  // ==================== PERIODIC ====================

  @Override
  public void periodic() {
    intakeRoller.updateTelemetry();
    intakePivot.updateTelemetry();
    Logger.recordOutput("Intake/PivotAngleDegrees", getPivotAngle().in(Degrees));
    Logger.recordOutput("Intake/IsDeployed", isDeployed());
  }

  @Override
  public void simulationPeriodic() {
    intakeRoller.simIterate();
    intakePivot.simIterate();
  }
}
