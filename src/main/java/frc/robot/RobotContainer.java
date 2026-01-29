// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
        public final DriveSubsystem m_robotDrive = new DriveSubsystem();
        private final SendableChooser<Command> autoChooser;
        CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

        public RobotContainer() {
                // Drive Subsystem Commands
                NamedCommands.registerCommand("setX", m_robotDrive.setXCommand());
                NamedCommands.registerCommand("zeroHeading", m_robotDrive.zeroHeadingCommand());
                configureButtonBindings();
                autoChooser = AutoBuilder.buildAutoChooser("None");
                SmartDashboard.putData("Auto Chooser", autoChooser);

                m_robotDrive.setDefaultCommand(
                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                new RunCommand(
                                                () -> m_robotDrive.drive(
                                                                -MathUtil.applyDeadband(
                                                                                m_driverController.getLeftY(),
                                                                                OIConstants.kDriveDeadband)
                                                                                * OIConstants.kDriveSpeedFactor,
                                                                -MathUtil.applyDeadband(
                                                                                m_driverController.getLeftX(),
                                                                                OIConstants.kDriveDeadband)
                                                                                * OIConstants.kDriveSpeedFactor,
                                                                -MathUtil.applyDeadband(
                                                                                m_driverController.getRightX(),
                                                                                OIConstants.kDriveDeadband),
                                                                true),
                                                m_robotDrive));
        }

        private void configureButtonBindings() {
                // Left Stick Button -> Set swerve to X
                m_driverController.leftStick().whileTrue(m_robotDrive.setXCommand());

                // Start Button -> Zero swerve heading
                m_driverController.start().onTrue(m_robotDrive.zeroHeadingCommand());

        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}
