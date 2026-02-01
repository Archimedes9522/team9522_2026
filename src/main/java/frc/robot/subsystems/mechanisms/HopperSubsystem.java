// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;
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
 * Hopper subsystem for FUEL ball storage and queuing.
 * 
 * <p>The hopper stores FUEL balls between the intake and shooter.
 * Uses beam break sensors to detect ball positions.
 * 
 * <p>Hardware:
 * <ul>
 *   <li>1x NEO motor with 4:1 reduction</li>
 *   <li>Entry beam break sensor (optional)</li>
 *   <li>Exit beam break sensor (optional)</li>
 * </ul>
 */
public class HopperSubsystem extends SubsystemBase {

  // === MOTORS ===
  private final SparkMax hopperMotor;

  // === SENSORS ===
  private final DigitalInput entryBeamBreak;
  private final DigitalInput exitBeamBreak;

  // === YAMS CONTROLLER ===
  private final SmartMotorController motorController;
  private final FlyWheel hopper;

  /**
   * Creates a new HopperSubsystem.
   */
  public HopperSubsystem() {
    // Initialize motor
    hopperMotor = new SparkMax(HopperConstants.kMotorId, MotorType.kBrushless);

    // Initialize beam break sensors
    entryBeamBreak = new DigitalInput(HopperConstants.kEntryBeamBreakPort);
    exitBeamBreak = new DigitalInput(HopperConstants.kExitBeamBreakPort);

    // Configure YAMS SmartMotorController
    SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.OPEN_LOOP)
        .withTelemetry("HopperMotor", TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(4)))  // 4:1 reduction
        .withMotorInverted(true)
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(HopperConstants.kCurrentLimitAmps));

    motorController = new SparkWrapper(hopperMotor, DCMotor.getNEO(1), smcConfig);

    // Configure YAMS FlyWheel
    FlyWheelConfig hopperConfig = new FlyWheelConfig(motorController)
        .withDiameter(Inches.of(4))
        .withMass(Pounds.of(0.5))
        .withUpperSoftLimit(RPM.of(6000))
        .withLowerSoftLimit(RPM.of(-6000))
        .withTelemetry("Hopper", TelemetryVerbosity.HIGH);

    hopper = new FlyWheel(hopperConfig);
  }

  // ==================== COMMANDS ====================

  /**
   * Runs the hopper forward to feed balls toward the shooter.
   * 
   * @return Command that runs while held
   */
  public Command feedCommand() {
    return hopper.set(HopperConstants.kFeedSpeed)
        .finallyDo(() -> motorController.setDutyCycle(0))
        .withName("Hopper.Feed");
  }

  /**
   * Runs the hopper in reverse for unjamming.
   * 
   * @return Command that runs while held
   */
  public Command reverseCommand() {
    return hopper.set(HopperConstants.kReverseSpeed)
        .finallyDo(() -> motorController.setDutyCycle(0))
        .withName("Hopper.Reverse");
  }

  /**
   * Stops the hopper.
   * 
   * @return Command that stops the hopper
   */
  public Command stopCommand() {
    return hopper.set(0).withName("Hopper.Stop");
  }

  // ==================== GETTERS ====================

  /**
   * Checks if a ball is at the entry sensor.
   * 
   * @return True if ball detected at entry
   */
  public boolean hasBallAtEntry() {
    // Beam break returns false when broken (ball present)
    return !entryBeamBreak.get();
  }

  /**
   * Checks if a ball is at the exit sensor (ready to feed to shooter).
   * 
   * @return True if ball detected at exit
   */
  public boolean hasBallAtExit() {
    return !exitBeamBreak.get();
  }

  /**
   * Checks if the hopper has any balls.
   * 
   * @return True if ball detected at either sensor
   */
  public boolean hasBall() {
    return hasBallAtEntry() || hasBallAtExit();
  }

  // ==================== PERIODIC ====================

  @Override
  public void periodic() {
    hopper.updateTelemetry();
    Logger.recordOutput("Hopper/BallAtEntry", hasBallAtEntry());
    Logger.recordOutput("Hopper/BallAtExit", hasBallAtExit());
  }

  @Override
  public void simulationPeriodic() {
    hopper.simIterate();
  }
}
