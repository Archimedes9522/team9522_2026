/**
 * Commands package for custom robot commands.
 * 
 * <p>This package contains reusable commands that can be bound to buttons
 * or used in autonomous routines. Commands encapsulate robot actions and
 * can be composed together using command groups.
 * 
 * <p><b>Command Types:</b>
 * <ul>
 *   <li>Instant commands - Run once and finish immediately</li>
 *   <li>Run commands - Run continuously until interrupted</li>
 *   <li>Sequential groups - Run commands one after another</li>
 *   <li>Parallel groups - Run commands simultaneously</li>
 * </ul>
 * 
 * <p><b>Example command structure:</b>
 * <pre>
 * public class ExampleCommand extends Command {
 *   private final ExampleSubsystem subsystem;
 *   
 *   public ExampleCommand(ExampleSubsystem subsystem) {
 *     this.subsystem = subsystem;
 *     addRequirements(subsystem);
 *   }
 *   
 *   {@literal @}Override
 *   public void initialize() { }
 *   
 *   {@literal @}Override
 *   public void execute() { }
 *   
 *   {@literal @}Override
 *   public boolean isFinished() { return false; }
 *   
 *   {@literal @}Override
 *   public void end(boolean interrupted) { }
 * }
 * </pre>
 * 
 * @see edu.wpi.first.wpilibj2.command.Command
 * @see <a href="https://docs.wpilib.org/en/stable/docs/software/commandbased/commands.html">WPILib Commands Documentation</a>
 */
package frc.robot.commands;
