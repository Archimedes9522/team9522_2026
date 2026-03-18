// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

/**
 * Subsystem to control an addressable LED strip (WS2812B).
 * Manages various visual states to communicate robot status.
 */
public class LEDSubsystem extends SubsystemBase {
  
  public enum LEDState {
    DISABLED_BREATHING(0),     // Alliance color breathing
    DEFAULT_TELEOP(1),         // Solid alliance color
    INTAKING(2),               // Scrolling orange
    HAS_PIECE(3),              // Solid orange
    AUTO_AIM_READY(4);         // Fast blinking green

    public final int priority;
    LEDState(int priority) { this.priority = priority; }
  }

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;
  
  private LEDState m_currentState = LEDState.DISABLED_BREATHING;
  private double m_lastStateChangeTime = 0;

  // Animation state
  private int m_chaseOffset = 0;

  public LEDSubsystem() {
    m_led = new AddressableLED(LEDConstants.kPwmPort);
    m_ledBuffer = new AddressableLEDBuffer(LEDConstants.kStripLength);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  /**
   * Sets the required state for the LEDs.
   */
  public void setState(LEDState state) {
    if (m_currentState != state) {
      m_currentState = state;
      m_lastStateChangeTime = Timer.getFPGATimestamp();
    }
  }

  @Override
  public void periodic() {
    switch (m_currentState) {
      case DISABLED_BREATHING:
        setBreathing(getAllianceColor());
        break;
      case DEFAULT_TELEOP:
        setSolid(getAllianceColor());
        break;
      case INTAKING:
        setChasing(Color.kOrange);
        break;
      case HAS_PIECE:
        setSolid(Color.kOrange);
        break;
      case AUTO_AIM_READY:
        setBlinking(Color.kGreen, 0.1); // 10Hz fast blink
        break;
    }
    m_led.setData(m_ledBuffer);
  }

  // ==================== ANIMATIONS ====================

  private void setSolid(Color color) {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, color);
    }
  }

  private void setBlinking(Color color, double blinkPeriodSec) {
    // Math.IEEEremainder gives a smooth blink if we use Math.sin, 
    // but for sharp on/off we just check the remainder of time
    double time = Timer.getFPGATimestamp();
    boolean isOn = (time % blinkPeriodSec) < (blinkPeriodSec / 2.0);
    
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      if (isOn) {
        m_ledBuffer.setLED(i, color);
      } else {
        m_ledBuffer.setRGB(i, 0, 0, 0); // Off
      }
    }
  }

  private void setBreathing(Color color) {
    // Pulse every 2 seconds
    double time = Timer.getFPGATimestamp();
    // Use cosine to scale from 0.05 to 1.0 (prevent complete off)
    double intensity = (Math.cos(time * Math.PI) + 1.0) / 2.0; 
    intensity = 0.05 + (intensity * 0.95);

    int r = (int)(color.red * 255 * intensity);
    int g = (int)(color.green * 255 * intensity);
    int b = (int)(color.blue * 255 * intensity);

    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, r, g, b);
    }
  }

  private void setChasing(Color color) {
    // Update offset roughly every 50ms for smooth chase
    double time = Timer.getFPGATimestamp();
    if (time - m_lastStateChangeTime > 0.05) {
      m_chaseOffset++;
      m_lastStateChangeTime = time;
    }

    int length = m_ledBuffer.getLength();
    for (int i = 0; i < length; i++) {
      // Create groups of 3 LEDs on, 3 LEDs off
      if ((i + m_chaseOffset) % 6 < 3) {
        m_ledBuffer.setLED(i, color);
      } else {
        m_ledBuffer.setRGB(i, 0, 0, 0);
      }
    }
  }

  private Color getAllianceColor() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      return Color.kRed;
    }
    return Color.kBlue;
  }
}
