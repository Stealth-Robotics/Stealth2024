// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AimAndShootCommand;
import frc.robot.commands.ReadyShooter;
import frc.robot.commands.defaultCommands.ClimberDefault;
import frc.robot.commands.defaultCommands.IntakeDefaultCommand;
import frc.robot.commands.defaultCommands.SwerveDriveTeleop;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.shooter.DistanceToShotValuesMap;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.PoseEstimationSystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  DistanceToShotValuesMap distanceToShotValuesMap = new DistanceToShotValuesMap();

  private final PoseEstimationSystem poseEstimationSystem = new PoseEstimationSystem();
  private final SwerveDrive swerveSubsystem = new SwerveDrive(poseEstimationSystem);
  private final RotatorSubsystem rotatorSubsystem = new RotatorSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem(
      () -> rotatorSubsystem.isHomed(),
      () -> (rotatorSubsystem.getMotorMode() == NeutralModeValue.Brake),
      () -> intake.isRingFullyInsideIntake());
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();

  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new
  CommandXboxController(1);

  public RobotContainer() {
    

    // Command Suppliers

    DoubleSupplier swerveTranslationYSupplier = () -> adjustInput(driverController.getLeftY());
    DoubleSupplier swerveTranslationXSupplier = () -> adjustInput(driverController.getLeftX());
    DoubleSupplier swerveRotationSupplier = () -> -adjustInput(driverController.getRightX());
    BooleanSupplier swerveHeadingResetBooleanSupplier = driverController.povDown();
    BooleanSupplier swerveRobotOrientedSupplier = driverController.rightBumper();

    BooleanSupplier rotatorHomeButtonSupplier = () -> rotatorSubsystem.getHomeButton();
    BooleanSupplier rotatorToggleMotorModeButtonSupplier = () -> rotatorSubsystem.getToggleMotorModeButton();

    DoubleSupplier intakeManualControlSupplier = () -> driverController.getRightTriggerAxis()
        - driverController.getLeftTriggerAxis();

    DoubleSupplier climberManuaControlSupplier = () -> operatorController.getRightTriggerAxis()
        - operatorController.getLeftTriggerAxis();

    // Default Commands

    swerveSubsystem.setDefaultCommand(new SwerveDriveTeleop(
        swerveSubsystem,
        swerveTranslationYSupplier,
        swerveTranslationXSupplier,
        swerveRotationSupplier,
        swerveRobotOrientedSupplier));

    intake.setDefaultCommand(new IntakeDefaultCommand(intake, intakeManualControlSupplier));
    climber.setDefaultCommand(new ClimberDefault(climberManuaControlSupplier, climber));

    // Driver Button Commands

    new Trigger(swerveHeadingResetBooleanSupplier).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));

    // Onboard Button Commands

    new Trigger(rotatorHomeButtonSupplier).onTrue(
        rotatorSubsystem.homeArmCommand().andThen(ledSubsystem.updateDisabledLEDsCommand()));

    new Trigger(rotatorToggleMotorModeButtonSupplier).onTrue(
        rotatorSubsystem.toggleMotorModeCommand().andThen(ledSubsystem.updateDisabledLEDsCommand()));

    // Other Triggers
    new Trigger(() -> intake.isRingFullyInsideIntake()).onTrue(ledSubsystem.blinkForRingCommand().andThen(new PrintCommand("ring")));
    new Trigger(() -> !intake.isRingFullyInsideIntake()).onTrue(ledSubsystem.idleCommand());

    new Trigger(driverController.leftBumper()).onTrue(new AimAndShootCommand(swerveSubsystem, rotatorSubsystem, shooter, intake, distanceToShotValuesMap));
    new Trigger(driverController.a()).onTrue(new InstantCommand(() -> shooter.stopShooterMotors()).alongWith(rotatorSubsystem.rotateToPositionCommand(Units.degreesToRotations(0))));
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
    ledSubsystem.idle();
  }

  public void disabledExit() {
    onInit();
  }

  public void disabledInit() {
    ledSubsystem.updateDisabledLEDs();
  }
}