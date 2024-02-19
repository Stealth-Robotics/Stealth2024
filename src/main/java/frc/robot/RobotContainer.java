// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AimAndShootCommand;
import frc.robot.commands.defaultCommands.IntakeDefaultCommand;
import frc.robot.commands.defaultCommands.SwerveDriveTeleop;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  private final SwerveDrive swerveSubsystem = new SwerveDrive();
  private final RotatorSubsystem rotatorSubsystem = new RotatorSubsystem();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem(
      () -> rotatorSubsystem.isHomed(),
      () -> rotatorSubsystem.getMotorMode() == NeutralModeValue.Brake);
  IntakeSubsystem intake = new IntakeSubsystem();

  private final CommandXboxController driverController = new CommandXboxController(0);
  private final ShooterSubsystem shooter = new ShooterSubsystem();

  IntakeSubsystem intake = new IntakeSubsystem();

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
        () -> -adjustInput(driverController.getLeftY()),
        () -> -adjustInput(driverController.getLeftX()),
        () -> adjustInput(driverController.getRightX()),
        () -> false));
      intake.setDefaultCommand(new IntakeDefaultCommand(intake, () -> (driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis())));

      new Trigger(driverController.a()).onTrue(new InstantCommand(() -> shooter.setMax()));
      new Trigger(driverController.b()).onTrue(new InstantCommand(() -> shooter.stopShooterMotors()));

      new Trigger(driverController.leftBumper()).onTrue(new AimAndShootCommand(swerveSubsystem, shooter, intake));
      // new Trigger(driverController.y()).onTrue()
      

  }

  

  private double adjustInput(double input) {
    return Math.copySign(Math.pow(Math.abs(input), 1.75), input);
  }

  public Command getAutonomousCommand() {
    return swerveSubsystem.followPathCommand("testPath", true);
  }

  public void autonomousInit() {
    rotatorSubsystem.holdCurrentPosition();
  }

  public void teleopInit() {
    rotatorSubsystem.holdCurrentPosition();
  }

  public void testInit() {
    rotatorSubsystem.holdCurrentPosition();
  }
}