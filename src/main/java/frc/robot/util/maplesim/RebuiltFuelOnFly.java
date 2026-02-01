// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.maplesim;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.utils.FieldMirroringUtils;
import org.littletonrobotics.junction.Logger;

/**
 * Represents a FUEL ball launched into the air.
 * 
 * <p>This class models a {@link RebuiltFuelOnField} that has been shot by the robot.
 * The simulation calculates projectile trajectory and determines if the FUEL hits the hub.
 * 
 * <p>Use {@link #setHitNetCallBack(Runnable)} to specify a callback that triggers
 * when the FUEL successfully enters the hub.
 */
public class RebuiltFuelOnFly extends GamePieceProjectile {

  /** Red hub position on the field (target for shooting) */
  private static final Translation3d RED_HUB_POSITION = new Translation3d(11.938, 4.034536, 1.5748);

  /**
   * Creates a new FUEL projectile.
   * 
   * @param robotPosition The robot's position on the field
   * @param shooterPositionOnRobot The shooter's offset from robot center
   * @param chassisSpeeds The robot's current velocity (for lead compensation)
   * @param shooterFacing The direction the shooter is facing
   * @param initialHeight The height of the shooter
   * @param launchingSpeed The speed at which the FUEL is launched
   * @param shooterAngle The angle of the shooter (pitch)
   */
  public RebuiltFuelOnFly(
      Translation2d robotPosition,
      Translation2d shooterPositionOnRobot,
      ChassisSpeeds chassisSpeeds,
      Rotation2d shooterFacing,
      Distance initialHeight,
      LinearVelocity launchingSpeed,
      Angle shooterAngle) {
    super(
        RebuiltFuelOnField.REBUILT_FUEL_INFO,
        robotPosition,
        shooterPositionOnRobot,
        chassisSpeeds,
        shooterFacing,
        initialHeight,
        launchingSpeed,
        shooterAngle);

    // FUEL touches ground at 3 inches (radius)
    super.withTouchGroundHeight(Inches.of(3).in(Meters));

    // Target is the hub (auto-mirrors for alliance)
    super.withTargetPosition(
        () -> FieldMirroringUtils.toCurrentAllianceTranslation(RED_HUB_POSITION));

    Logger.recordOutput("HubGoal", RED_HUB_POSITION);

    // Tolerance for hitting the hub (23.5" x 23.5" opening, 1" height tolerance)
    super.withTargetTolerance(
        new Translation3d(
            Inches.of(23.5).in(Meters),
            Inches.of(23.5).in(Meters),
            Inches.of(1).in(Meters)));

    // If the FUEL misses, it becomes a ground game piece
    super.enableBecomesGamePieceOnFieldAfterTouchGround();
  }
}
