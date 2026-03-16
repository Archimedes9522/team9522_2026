package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Field coordinates and dimension constants for the 2026 "Rebuilt" game.
 */
public final class FieldConstants {

  /**
   * Aim points for the 2026 "Rebuilt" game field.
   * 
   * <p>These are field positions in meters that mechanisms can aim at.
   * The field is 16.54m x 8.05m. Red alliance is on the right side (high X).
   * 
   * <p>Each enum value contains a Translation3d (X, Y, Z) in meters.
   */
  public enum AimPoints {
    /** Red alliance hub (scoring target) - right side of field */
    RED_HUB(new Translation3d(11.938, 4.034536, 1.5748)),
    
    /** Red alliance outpost position - moved inward toward center */
    RED_OUTPOST(new Translation3d(12.5, 6.0, 0)),
    
    /** Red alliance far side position - moved inward toward center */
    RED_FAR_SIDE(new Translation3d(12.5, 2.0, 0)),

    /** Blue alliance hub (scoring target) - left side of field */
    BLUE_HUB(new Translation3d(4.5974, 4.034536, 1.5748)),
    
    /** Blue alliance outpost position - moved inward toward center */
    BLUE_OUTPOST(new Translation3d(4.0, 2.0, 0)),
    
    /** Blue alliance far side position - moved inward toward center */
    BLUE_FAR_SIDE(new Translation3d(4.0, 6.0, 0));

    /** The 3D position of this aim point on the field */
    public final Translation3d value;

    private AimPoints(Translation3d value) {
      this.value = value;
    }

    /**
     * Gets the hub position for the current alliance.
     * @return Hub Translation3d for red or blue based on DriverStation alliance
     */
    public static Translation3d getAllianceHubPosition() {
      return DriverStation.getAlliance()
          .map(alliance -> alliance == DriverStation.Alliance.Red ? RED_HUB.value : BLUE_HUB.value)
          .orElse(RED_HUB.value);
    }

    /**
     * Gets the outpost position for the current alliance.
     * @return Outpost Translation3d for red or blue based on DriverStation alliance
     */
    public static Translation3d getAllianceOutpostPosition() {
      return DriverStation.getAlliance()
          .map(alliance -> alliance == DriverStation.Alliance.Red ? RED_OUTPOST.value : BLUE_OUTPOST.value)
          .orElse(RED_OUTPOST.value);
    }

    /**
     * Gets the far side position for the current alliance.
     * @return Far side Translation3d for red or blue based on DriverStation alliance
     */
    public static Translation3d getAllianceFarSidePosition() {
      return DriverStation.getAlliance()
          .map(alliance -> alliance == DriverStation.Alliance.Red ? RED_FAR_SIDE.value : BLUE_FAR_SIDE.value)
          .orElse(RED_FAR_SIDE.value);
    }
  }
}
