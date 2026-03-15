// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.system.plant.DCMotor;
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
 * Beam break sensors are not installed on this robot.
 * 
 * <p>Hardware:
 * <ul>
 *   <li>1x NEO Vortex motor (SparkFlex) with 4:1 reduction</li>
 *   <li>No ball sensors installed</li>
 * </ul>
 */
public class HopperSubsystem extends SubsystemBase {

  // === MOTORS ===
  private final SparkFlex hopperMotor;

  // === YAMS CONTROLLER ===
  private final SmartMotorController motorController;
  private final FlyWheel hopper;

  /**
   * Creates a new HopperSubsystem.
   */
  public HopperSubsystem() {
    // Initialize motor
    hopperMotor = new SparkFlex(HopperConstants.kMotorId, MotorType.kBrushless);

    // Configure YAMS SmartMotorController
    SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.OPEN_LOOP)
        .withTelemetry("HopperMotor", TelemetryVerbosity.HIGH)
  .withGearing(new MechanismGearing(GearBox.fromReductionStages(HopperConstants.kGearRatio)))
        .withMotorInverted(true)
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(HopperConstants.kCurrentLimitAmps));

    motorController = new SparkWrapper(hopperMotor, DCMotor.getNeoVortex(1), smcConfig);

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

  // ==================== PERIODIC ====================

  @Override
  public void periodic() {
    hopper.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    hopper.simIterate();
  }
}
