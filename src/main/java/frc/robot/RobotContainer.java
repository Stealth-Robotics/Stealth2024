// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.defaultCommands.SwerveDriveTeleop;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.PoseEstimationSystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  private final PoseEstimationSystem poseEstimationSystem = new PoseEstimationSystem();
  private final SwerveDrive swerveSubsystem = new SwerveDrive(poseEstimationSystem);

  private final CommandXboxController driverController = new CommandXboxController(0);

  public RobotContainer() {

    configureBindings();
    swerveSubsystem.setDefaultCommand(new SwerveDriveTeleop(
        swerveSubsystem,
        () -> -driverController.getLeftY(),
        () -> driverController.getLeftX(),
        () -> driverController.getRightX(),
        () -> false));

  }

  private void configureBindings() {
    driverController.povDown().onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
  }

  public Command getAutonomousCommand() {
    return new PrintCommand("Hello, world!");
  }
}
