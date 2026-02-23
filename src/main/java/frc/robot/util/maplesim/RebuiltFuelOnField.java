// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.maplesim;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.dyn4j.geometry.Circle;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

/**
 * Represents a FUEL game piece in the 2026 "Rebuilt" game.
 * 
 * <p>FUEL balls are 15cm (6") diameter spheres that can be:
 * <ul>
 *   <li>Picked up from the field by the intake</li>
 *   <li>Stored in the hopper</li>
 *   <li>Shot into the hub for points</li>
 * </ul>
 */
public class RebuiltFuelOnField extends GamePieceOnFieldSimulation {
  
  /** 
   * Game piece info for FUEL balls.
   * - 7.5cm radius (15cm diameter)
   * - 15cm height
   * - 0.5 lbs weight
   * - Friction and restitution coefficients
   */
  public static final GamePieceInfo REBUILT_FUEL_INFO = new GamePieceInfo(
      "Fuel",
      new Circle(Centimeters.of(7.5).in(Meters)),
      Centimeter.of(15),
      Pounds.of(0.5),
      1.8,  // friction coefficient
      5,    // linear damping
      0.8   // restitution (bounciness)
  );

  /**
   * Creates a new FUEL game piece at the specified position.
   * 
   * @param initialPosition The field position to spawn the FUEL
   */
  public RebuiltFuelOnField(Translation2d initialPosition) {
    super(REBUILT_FUEL_INFO, new Pose2d(initialPosition, new Rotation2d()));
  }
}
