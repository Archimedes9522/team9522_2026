// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KickerConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

/**
 * Kicker subsystem to feed FUEL balls from hopper to shooter.
 * 
 * <p>The kicker is a fast roller that accelerates balls into the shooter.
 * It's the final stage before the ball enters the flywheel.
 * 
 * <p>Hardware:
 * <ul>
 *   <li>1x NEO motor with 4:1 reduction</li>
 * </ul>
 */
public class KickerSubsystem extends SubsystemBase {

  // === MOTORS ===
  private final SparkMax kickerMotor;

  // === YAMS CONTROLLER ===
  private final SmartMotorController motorController;
  private final FlyWheel kicker;

  /**
   * Creates a new KickerSubsystem.
   */
  public KickerSubsystem() {
    // Initialize motor
    kickerMotor = new SparkMax(KickerConstants.kMotorId, MotorType.kBrushless);

    // Configure YAMS SmartMotorController
    SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.OPEN_LOOP)
        .withTelemetry("KickerMotor", TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(4)))  // 4:1 reduction
        .withMotorInverted(true)
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(KickerConstants.kCurrentLimitAmps));

    motorController = new SparkWrapper(kickerMotor, DCMotor.getNEO(1), smcConfig);

    // Configure YAMS FlyWheel
    FlyWheelConfig kickerConfig = new FlyWheelConfig(motorController)
        .withDiameter(Inches.of(4))
        .withMass(Pounds.of(0.5))
        .withUpperSoftLimit(RPM.of(6000))
        .withLowerSoftLimit(RPM.of(-6000))
        .withTelemetry("Kicker", TelemetryVerbosity.HIGH);

    kicker = new FlyWheel(kickerConfig);
  }

  // ==================== COMMANDS ====================

  /**
   * Runs the kicker to feed balls to the shooter.
   * 
   * @return Command that runs while held
   */
  public Command feedCommand() {
    return kicker.set(KickerConstants.kFeedSpeed)
        .finallyDo(() -> motorController.setDutyCycle(0))
        .withName("Kicker.Feed");
  }

  /**
   * Runs the kicker in reverse for unjamming.
   * 
   * @return Command that runs while held
   */
  public Command reverseCommand() {
    return kicker.set(-KickerConstants.kFeedSpeed)
        .finallyDo(() -> motorController.setDutyCycle(0))
        .withName("Kicker.Reverse");
  }

  /**
   * Stops the kicker.
   * 
   * @return Command that stops the kicker
   */
  public Command stopCommand() {
    return kicker.set(0).withName("Kicker.Stop");
  }

  // ==================== PERIODIC ====================

  @Override
  public void periodic() {
    kicker.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    kicker.simIterate();
  }
}
