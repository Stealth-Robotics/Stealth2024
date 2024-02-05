// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.defaultCommands.RotatorDefaultCommand;
import frc.robot.commands.defaultCommands.SwerveDriveTeleop;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  private final SwerveDrive swerveSubsystem = new SwerveDrive();
  private final RotatorSubsystem rotatorSubsystem = new RotatorSubsystem();

  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  public RobotContainer() {

    // Command Suppliers
    DoubleSupplier swerveTranslationYSupplier = () -> -adjustInput(driverController.getLeftY());
    DoubleSupplier swerveTranslationXSupplier = () -> -adjustInput(driverController.getLeftX());
    DoubleSupplier swerveRotationSupplier = () -> adjustInput(driverController.getRightX());
    BooleanSupplier swerveHeadingResetBooleanSupplier = driverController.povDown();
    BooleanSupplier swerveRobotOrientedSupplier = driverController.rightBumper();

    DoubleSupplier rotatorManualControlSupplier = () -> operatorController.getLeftY();
    BooleanSupplier rotatorHomeButtonSupplier = () -> rotatorSubsystem.getHomeButton();
    BooleanSupplier rotatorToggleMotorModeButtonSupplier = () -> rotatorSubsystem.getToggleMotorModeButton();

    DoubleSupplier intakeManualControlSupplier = () -> driverController.getRightTriggerAxis()
        - driverController.getLeftTriggerAxis();

    // Default Commands

    swerveSubsystem.setDefaultCommand(new SwerveDriveTeleop(
        swerveSubsystem,
        swerveTranslationYSupplier,
        swerveTranslationXSupplier,
        swerveRotationSupplier,
        swerveRobotOrientedSupplier));

    rotatorSubsystem.setDefaultCommand(new RotatorDefaultCommand(rotatorSubsystem, rotatorManualControlSupplier));

    // Gamepad Button Commands

    new Trigger(swerveHeadingResetBooleanSupplier).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));

    // Onboard Robot Button Commands

    new Trigger(rotatorHomeButtonSupplier).onTrue(
        rotatorSubsystem.homeArmCommand());

    new Trigger(rotatorToggleMotorModeButtonSupplier).onTrue(
        rotatorSubsystem.toggleMotorModeCommand());
  }

  private double adjustInput(double input) {
    return Math.copySign(Math.pow(Math.abs(input), 1.75), input);
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
