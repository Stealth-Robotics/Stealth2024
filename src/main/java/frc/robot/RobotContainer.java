// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.defaultCommands.SwerveDriveTeleop;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  private final SwerveDrive swerveSubsystem = new SwerveDrive();
  private final RotatorSubsystem rotatorSubsystem = new RotatorSubsystem();

  private final CommandXboxController driverController = new CommandXboxController(0);

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveDriveTeleop(
        swerveSubsystem,
        () -> -adjustInput(driverController.getLeftY()),
        () -> -adjustInput(driverController.getLeftX()),
        () -> adjustInput(driverController.getRightX()),
        () -> false));

    configureBindings();
  }

  private double adjustInput(double input) {
    return Math.copySign(Math.pow(Math.abs(input), 1.75), input);
  }

  private void configureBindings() {
    driverController.povDown().onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));

    new Trigger(() -> rotatorSubsystem.getHomeButton()).onTrue(
        rotatorSubsystem.homeArmCommand());

    new Trigger(() -> rotatorSubsystem.getToggleMotorModeButton()).onTrue(
        rotatorSubsystem.toggleMotorModeCommand());
  }

  public Command getAutonomousCommand() {
    return swerveSubsystem.followPathCommand("testPath", true);
  }

  public void autonomousInit() {
    rotatorSubsystem.holdCurrentPosition();
    ;
  }

  public void teleopInit() {
    rotatorSubsystem.holdCurrentPosition();
    ;
  }
}
