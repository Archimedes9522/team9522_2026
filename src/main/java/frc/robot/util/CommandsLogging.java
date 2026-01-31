// Copyright (c) 2026 FRC 167
// Adapted for Team 9522
// Original: https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Utility for logging running commands to AdvantageKit/AdvantageScope.
 * 
 * <p>This provides visibility into what commands are running at any given time,
 * which is invaluable for debugging complex command compositions.
 * 
 * <p>Usage: Call {@link #initialize()} once in Robot.robotInit(), then call
 * {@link #logCommands()} in Robot.robotPeriodic().
 * 
 * <p>In AdvantageScope, you'll see:
 * <ul>
 *   <li>Commands/Running - List of currently running command names</li>
 *   <li>Commands/Scheduled - Total count of scheduled commands</li>
 *   <li>Commands/Subsystems - Which subsystems are in use</li>
 * </ul>
 * 
 * <p>Adapted from FRC 167's CommandsLogging utility.
 */
public class CommandsLogging {
  
  /** Map tracking which command is using each subsystem */
  private static final Map<Subsystem, Command> subsystemCommands = new HashMap<>();
  
  /** Whether logging has been initialized */
  private static boolean initialized = false;

  /**
   * Initializes command logging by registering callbacks with the CommandScheduler.
   * Call this once in Robot.robotInit().
   */
  public static void initialize() {
    if (initialized) {
      return;
    }
    
    CommandScheduler scheduler = CommandScheduler.getInstance();
    
    // Log when commands are initialized (started)
    scheduler.onCommandInitialize(command -> {
      Logger.recordOutput("Commands/Events/Started", command.getName());
      
      // Track subsystem usage
      for (Subsystem subsystem : command.getRequirements()) {
        subsystemCommands.put(subsystem, command);
      }
    });
    
    // Log when commands finish
    scheduler.onCommandFinish(command -> {
      Logger.recordOutput("Commands/Events/Finished", command.getName());
      
      // Clear subsystem tracking
      for (Subsystem subsystem : command.getRequirements()) {
        subsystemCommands.remove(subsystem);
      }
    });
    
    // Log when commands are interrupted
    scheduler.onCommandInterrupt(command -> {
      Logger.recordOutput("Commands/Events/Interrupted", command.getName());
      
      // Clear subsystem tracking
      for (Subsystem subsystem : command.getRequirements()) {
        subsystemCommands.remove(subsystem);
      }
    });
    
    initialized = true;
    Logger.recordOutput("Commands/Initialized", true);
  }

  /**
   * Logs the current state of running commands.
   * Call this in Robot.robotPeriodic().
   */
  public static void logCommands() {
    if (!initialized) {
      return;
    }
    
    // Build list of subsystem -> command mappings
    List<String> subsystemUsage = new ArrayList<>();
    for (Map.Entry<Subsystem, Command> entry : subsystemCommands.entrySet()) {
      String subsystemName = entry.getKey().getClass().getSimpleName();
      String commandName = entry.getValue().getName();
      subsystemUsage.add(subsystemName + " -> " + commandName);
    }
    
    // Log to AdvantageKit
    Logger.recordOutput("Commands/SubsystemUsage", subsystemUsage.toArray(new String[0]));
    Logger.recordOutput("Commands/ActiveSubsystems", subsystemCommands.size());
  }

  /**
   * Gets a readable name for a command, including its type.
   * 
   * @param command The command to name
   * @return A descriptive name string
   */
  public static String getCommandName(Command command) {
    if (command == null) {
      return "null";
    }
    
    String name = command.getName();
    String type = command.getClass().getSimpleName();
    
    // If the name is the same as the class name, just return that
    if (name.equals(type)) {
      return name;
    }
    
    // Otherwise include both
    return name + " (" + type + ")";
  }

  /**
   * Checks if a subsystem currently has a command running.
   * 
   * @param subsystem The subsystem to check
   * @return True if the subsystem has an active command
   */
  public static boolean isSubsystemBusy(Subsystem subsystem) {
    return subsystemCommands.containsKey(subsystem);
  }

  /**
   * Gets the command currently using a subsystem.
   * 
   * @param subsystem The subsystem to check
   * @return The active command, or null if none
   */
  public static Command getSubsystemCommand(Subsystem subsystem) {
    return subsystemCommands.get(subsystem);
  }
}
