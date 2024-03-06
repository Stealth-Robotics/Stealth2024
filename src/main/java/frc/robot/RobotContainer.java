// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.autos.CenterTwoRing;
import frc.robot.autos.FourRingSourceSide;
import frc.robot.autos.ThreeRingAmpSide;
import frc.robot.autos.ThreeRingRightSide;
import frc.robot.autos.TwoRingSourceSide;
import frc.robot.commands.AimAndShootCommand;
import frc.robot.commands.AmpPresetCommand;
import frc.robot.commands.StowPreset;
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

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.platform.can.AutocacheState;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  DistanceToShotValuesMap distanceToShotValuesMap = new DistanceToShotValuesMap();

  private final SwerveDrive swerveSubsystem = new SwerveDrive();
  private final RotatorSubsystem rotatorSubsystem = new RotatorSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem(
      () -> rotatorSubsystem.isHomed(),
      () -> (rotatorSubsystem.getMotorMode() == NeutralModeValue.Brake),
      () -> intake.isRingFullyInsideIntake());
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();

  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  public RobotContainer() {

    autoChooser.addOption("Three ring source side",
        new ThreeRingRightSide(swerveSubsystem, rotatorSubsystem, shooter, intake));
    autoChooser.addOption("Two ring source side",
        new TwoRingSourceSide(swerveSubsystem, rotatorSubsystem, shooter, intake));
  autoChooser.addOption("Three ring amp side",
        new ThreeRingAmpSide(swerveSubsystem, rotatorSubsystem, shooter, intake));
    autoChooser.addOption("Four ring source side",
        new FourRingSourceSide(swerveSubsystem, rotatorSubsystem, shooter, intake));
    autoChooser.addOption("nothini", new InstantCommand());
    autoChooser.addOption("center two ring", new CenterTwoRing(swerveSubsystem, rotatorSubsystem, shooter, intake));
    SmartDashboard.putData("auto", autoChooser);

    // Command Suppliers

    DoubleSupplier swerveTranslationYSupplier = () -> -adjustInput(driverController.getLeftY());
    DoubleSupplier swerveTranslationXSupplier = () -> -adjustInput(driverController.getLeftX());
    DoubleSupplier swerveRotationSupplier = () -> -adjustInput(driverController.getRightX());
    BooleanSupplier swerveHeadingResetBooleanSupplier = driverController.povDown();
    BooleanSupplier swerveRobotOrientedSupplier = () -> false;

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
        swerveRobotOrientedSupplier,
        swerveSubsystem.isRed()));

    intake.setDefaultCommand(new IntakeDefaultCommand(intake, intakeManualControlSupplier));
    climber.setDefaultCommand(
        new ClimberDefault(() -> operatorController.getLeftY(), () -> operatorController.getRightY(), climber));

    // Driver Button Commands

    new Trigger(swerveHeadingResetBooleanSupplier).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));

    new Trigger(driverController.leftBumper())
        .onTrue(new AimAndShootCommand(swerveSubsystem, rotatorSubsystem, shooter, intake, distanceToShotValuesMap,
            swerveSubsystem.getDistanceOffsetSupplier()).andThen(new StowPreset(rotatorSubsystem, shooter)));

    new Trigger(driverController.a()).onTrue(new InstantCommand(() -> shooter.stopShooterMotors())
        .alongWith(rotatorSubsystem.rotateToPositionCommand(Units.degreesToRotations(0))));

    new Trigger(driverController.rightBumper()).onTrue(
        new AmpPresetCommand(rotatorSubsystem, shooter));

    // new Trigger(() -> Math.abs(operatorController.getLeftY()) > 0.1).onTrue(
    // rotatorSubsystem.armManualControl(() -> -operatorController.getLeftY(),
    // rotatorSubsystem.getRotatorState()));

    // subwoofer shot
    new Trigger(driverController.y()).onTrue(
        new AimAndShootCommand(swerveSubsystem, rotatorSubsystem, shooter, intake, distanceToShotValuesMap, 1.4));

    new Trigger(driverController.x()).onTrue(new InstantCommand(() -> shooter.setShooterDutyCycle(0.4)))
        .onFalse(new InstantCommand(() -> shooter.setShooterDutyCycle(0)));

    // Onboard Button Commands

    new Trigger(rotatorHomeButtonSupplier).onTrue(
        rotatorSubsystem.homeArmCommand().andThen(ledSubsystem.updateDisabledLEDsCommand()));

    new Trigger(rotatorToggleMotorModeButtonSupplier).onTrue(
        rotatorSubsystem.toggleMotorModeCommand().andThen(ledSubsystem.updateDisabledLEDsCommand()));

    // Other Triggers
    new Trigger(() -> intake.isRingAtFrontOfIntake())
        .onTrue(ledSubsystem.blinkForRingCommand().andThen(new PrintCommand("ring")));
    new Trigger(() -> !intake.isRingFullyInsideIntake()).onTrue(ledSubsystem.idleCommand());

  }

  private double adjustInput(double input) {
    return Math.copySign(Math.pow(Math.abs(input), 1.75), input);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    // return swerveSubsystem.followPathCommand("test", true);
  }

  public void onInit() {
    rotatorSubsystem.setMotorsToCoast();
    // swerveSubsystem.setCurrentAlliance();
    rotatorSubsystem.holdCurrentPosition();
    shooter.stopShooterMotors();
    ledSubsystem.idle();
  }

  public void disabledExit() {
    onInit();
  }

  public void disabledInit() {
    ledSubsystem.updateDisabledLEDs();
  }
}