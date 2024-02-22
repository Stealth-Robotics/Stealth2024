// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.defaultCommands.IntakeDefaultCommand;
import frc.robot.commands.defaultCommands.SwerveDriveTeleop;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.PoseEstimationSystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  private final PoseEstimationSystem poseEstimationSystem = new PoseEstimationSystem();
  private final SwerveDrive swerveSubsystem = new SwerveDrive(poseEstimationSystem);
  private final RotatorSubsystem rotatorSubsystem = new RotatorSubsystem();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem(
      () -> rotatorSubsystem.isHomed(),
      () -> rotatorSubsystem.getMotorMode() == NeutralModeValue.Brake);
  IntakeSubsystem intake = new IntakeSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();

  private final CommandXboxController driverController = new CommandXboxController(0);
  // private final CommandXboxController operatorController = new
  // CommandXboxController(1);

  public RobotContainer() {

    // Command Suppliers

    DoubleSupplier swerveTranslationYSupplier = () -> -adjustInput(driverController.getLeftY());
    DoubleSupplier swerveTranslationXSupplier = () -> -adjustInput(driverController.getLeftX());
    DoubleSupplier swerveRotationSupplier = () -> adjustInput(driverController.getRightX());
    BooleanSupplier swerveHeadingResetBooleanSupplier = driverController.povDown();
    BooleanSupplier swerveRobotOrientedSupplier = driverController.rightBumper();

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

    intake.setDefaultCommand(new IntakeDefaultCommand(intake, intakeManualControlSupplier));

    // Driver Button Commands

    new Trigger(swerveHeadingResetBooleanSupplier).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));

    new Trigger(driverController.a())
        .onTrue(rotatorSubsystem.rotateToPositionCommand(Units.radiansToRotations(Math.toRadians(45))));
    new Trigger(driverController.b())
        .onTrue(rotatorSubsystem.rotateToPositionCommand(Units.radiansToRotations(Math.toRadians(90))));
    new Trigger(driverController.x())
        .onTrue(rotatorSubsystem.rotateToPositionCommand(Units.radiansToRotations(Math.toRadians(0))));

    // Onboard Button Commands

    new Trigger(rotatorHomeButtonSupplier).onTrue(
        rotatorSubsystem.homeArmCommand().andThen(ledSubsystem.updateLEDsCommand()));

    new Trigger(rotatorToggleMotorModeButtonSupplier).onTrue(
        rotatorSubsystem.toggleMotorModeCommand().andThen(ledSubsystem.updateLEDsCommand()));
  }

  private double adjustInput(double input) {
    return Math.copySign(Math.pow(Math.abs(input), 1.75), input);
  }

  public Command getAutonomousCommand() {
    return swerveSubsystem.followPathCommand("testPath", true);
  }

  public void onInit() {
    swerveSubsystem.setTargetGoal();
    rotatorSubsystem.holdCurrentPosition();
  }

  public void disabledExit() {
    onInit();
  }
}