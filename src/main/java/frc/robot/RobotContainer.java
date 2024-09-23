// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.CenterAuto;
import frc.robot.autos.CenterFiveRing;
import frc.robot.autos.CenterTwoRing;
import frc.robot.autos.ThreeRingAmpSide;
import frc.robot.autos.ThreeRingRightSide;
import frc.robot.autos.TwoRingSourceSide;
import frc.robot.commands.AimAndShootCommand;
import frc.robot.commands.AlignToRing;
import frc.robot.commands.AmpPresetCommand;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.DriveWhileAlignedToSpeaker;
import frc.robot.commands.ReadyShooter;
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
import frc.robot.subsystems.vision.LimelightHelpers;

import java.time.Instant;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class RobotContainer {

        private final SendableChooser<Command> autoChooser = new SendableChooser<>();
        private final SwerveDrive swerveSubsystem = new SwerveDrive();
        private final RotatorSubsystem rotatorSubsystem = new RotatorSubsystem();
        private final IntakeSubsystem intake = new IntakeSubsystem();
        private final LEDSubsystem ledSubsystem = new LEDSubsystem(rotatorSubsystem::isHomed,
                        () -> (rotatorSubsystem.getMotorMode() == NeutralModeValue.Brake),
                        intake::isRingFullyInsideIntake);
        private final ShooterSubsystem shooter = new ShooterSubsystem();
        private final ClimberSubsystem climber = new ClimberSubsystem();

        private final DistanceToShotValuesMap map = new DistanceToShotValuesMap();
        private final CommandXboxController driverController = new CommandXboxController(0);
        private final CommandXboxController operatorController = new CommandXboxController(1);

        public RobotContainer() {

                autoChooser.addOption("Three ring source side",
                                new ThreeRingRightSide(swerveSubsystem, rotatorSubsystem, shooter, intake));
                autoChooser.addOption("Two ring source side",
                                new TwoRingSourceSide(swerveSubsystem, rotatorSubsystem, shooter, intake));
                autoChooser.addOption("Three ring amp side",
                                new ThreeRingAmpSide(swerveSubsystem, rotatorSubsystem, shooter, intake));
                autoChooser.addOption("Center 4 ring",
                                new CenterAuto(swerveSubsystem, rotatorSubsystem, shooter, intake));
                autoChooser.addOption("center 5 ring",
                                new CenterFiveRing(swerveSubsystem, rotatorSubsystem, shooter, intake));
                autoChooser.addOption("nothing", new InstantCommand());

                autoChooser.addOption("center", new CenterAuto(swerveSubsystem, rotatorSubsystem, shooter, intake));
                autoChooser.addOption("test",
                                new SequentialCommandGroup(
                                                new InstantCommand(() -> swerveSubsystem.setInitialPose("testPath")),
                                                swerveSubsystem.followPathCommand("testPath")));

                autoChooser.addOption("shoot only",
                                new ReadyShooter(shooter, rotatorSubsystem, intake, () -> 1.4)
                                                .andThen(intake.convey())
                                                .andThen(new StowPreset(rotatorSubsystem, shooter)));

                autoChooser.addOption("shoot preload drive out",
                                new ReadyShooter(shooter, rotatorSubsystem, intake, () -> 1.4)
                                                .andThen(intake.convey())
                                                .andThen(new StowPreset(rotatorSubsystem, shooter))
                                                .andThen(swerveSubsystem.followPathCommand("drive away")));
                autoChooser.addOption("center 2",
                                new CenterTwoRing(swerveSubsystem, rotatorSubsystem, shooter, intake));
                SmartDashboard.putData("auto", autoChooser);

                // Command Suppliers

                DoubleSupplier swerveTranslationYSupplier = () -> -adjustInput(driverController.getLeftY());
                DoubleSupplier swerveTranslationXSupplier = () -> -adjustInput(driverController.getLeftX());
                DoubleSupplier swerveRotationSupplier = () -> -adjustInput(driverController.getRightX());
                BooleanSupplier swerveHeadingResetBooleanSupplier = driverController.povDown();
                BooleanSupplier swerveRobotOrientedSupplier = driverController.x();

                BooleanSupplier rotatorHomeButtonSupplier = rotatorSubsystem::getHomeButton;
                BooleanSupplier rotatorToggleMotorModeButtonSupplier = rotatorSubsystem::getToggleMotorModeButton;

                DoubleSupplier intakeManualControlSupplier = () -> driverController.getRightTriggerAxis()
                                - driverController.getLeftTriggerAxis();

                // Default Commands

                swerveSubsystem.setDefaultCommand(
                                new SwerveDriveTeleop(swerveSubsystem, swerveTranslationYSupplier,
                                                swerveTranslationXSupplier,
                                                swerveRotationSupplier, swerveRobotOrientedSupplier,
                                                swerveSubsystem.isRed()));

                intake.setDefaultCommand(new IntakeDefaultCommand(intake, intakeManualControlSupplier));
                climber.setDefaultCommand(
                                new ClimberDefault(operatorController::getLeftY, operatorController::getRightY,
                                                climber));

                // Driver Button Commands

                new Trigger(swerveHeadingResetBooleanSupplier).onTrue(new InstantCommand(swerveSubsystem::zeroHeading));

                // new Trigger(driverController.leftBumper()).onTrue(new
                // AimAndShootCommand(swerveSubsystem, rotatorSubsystem,
                // shooter, intake, () -> swerveSubsystem.getDistanceMetersToGoal()));
                new Trigger(driverController.x().onTrue(new InstantCommand(() -> rotatorSubsystem.resetEncoder())));
                new Trigger(driverController.leftBumper()).onTrue(

                                swerveSubsystem.rotateToGoal()
                                                .andThen(new ReadyShooter(shooter, rotatorSubsystem,
                                                                intake,
                                                                () -> swerveSubsystem
                                                                                .getDistanceMetersToGoal()))
                                                .andThen(intake.convey())
                                                .andThen(new StowPreset(rotatorSubsystem, shooter)));

                new Trigger(driverController.a()).onTrue(
                                new StowPreset(rotatorSubsystem, shooter));

                new Trigger(driverController.povUp()).onTrue(new InstantCommand(rotatorSubsystem::resetEncoder));

                // new Trigger(driverController.x())
                // .onTrue(rotatorSubsystem.rotateToPositionCommand(() ->
                // Units.degreesToRotations(50)));

                new Trigger(driverController.rightBumper()).onTrue(
                                new ConditionalCommand(
                                                rotatorSubsystem.rotateToPositionCommand(
                                                                () -> Units.degreesToRotations(53)),
                                                new AmpPresetCommand(rotatorSubsystem, shooter, intake),
                                                rotatorSubsystem.getAmpOutIntake()));

                new Trigger(() -> Math.abs(operatorController.getRightTriggerAxis()
                                - operatorController.getLeftTriggerAxis()) > 0.1).onTrue(
                                                rotatorSubsystem.armManulControl(() -> (operatorController
                                                                .getRightTriggerAxis()
                                                                - operatorController.getLeftTriggerAxis())));

                // subwoofer shot
                new Trigger(driverController.y())
                                .onTrue(new ReadyShooter(shooter, rotatorSubsystem, intake, () -> 1.4)
                                                .andThen(intake.convey())
                                                .andThen(new StowPreset(rotatorSubsystem, shooter)));

                // new Trigger(driverController.x()).whileTrue(new
                // AlignToRing(swerveSubsystem));

                // new Trigger(driverController.b()).whileTrue(new
                // DriveWhileAlignedToSpeaker(swerveSubsystem,
                // swerveTranslationYSupplier, swerveTranslationXSupplier,
                // swerveSubsystem.isRed())
                // .alongWith(rotatorSubsystem.rotateWhileDrivingCommand(
                // () -> map.getInterpolatedRotationAngle(swerveSubsystem
                // .getDistanceMetersToGoal()))));

                new Trigger(driverController.b().onTrue(
                                new InstantCommand(() -> intake.setIntakeSpeed(-0.2))
                                                .andThen(new WaitCommand(0.1))
                                                .andThen(new InstantCommand(() -> intake.setIntakeSpeed(0)))
                                                .andThen(rotatorSubsystem.rotateToPositionCommand(
                                                                () -> Units.degreesToRotations(30), 2))
                                                .alongWith(shooter.spinToRps(() -> 60))
                                                .andThen(intake.convey())

                ));

                // Onboard Button Commands

                new Trigger(rotatorHomeButtonSupplier)
                                .onTrue(rotatorSubsystem.homeArmCommand()
                                                .andThen(ledSubsystem.updateDisabledLEDsCommand()));

                new Trigger(rotatorToggleMotorModeButtonSupplier)
                                .onTrue(rotatorSubsystem.toggleMotorModeCommand()
                                                .andThen(ledSubsystem.updateDisabledLEDsCommand()));

                // Other Triggers
                new Trigger(intake::isRingAtFrontOfIntake)
                                .onTrue(ledSubsystem.blinkForRingCommand().andThen(new PrintCommand("ring")));
                new Trigger(() -> (swerveSubsystem.seesNote() && !intake.isRingFullyInsideIntake()))
                                .onTrue(ledSubsystem.seesRingCommand());
        }

        private double adjustInput(double input) {
                return Math.copySign(Math.pow(Math.abs(input), 1.75), input);
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
                // return swerveSubsystem.followPathCommand("test", true);
        }

        public void onInit() {
                // swerveSubsystem.setCurrentAlliance();
                rotatorSubsystem.setMotorsToCoast();
                rotatorSubsystem.holdCurrentPosition();
                shooter.stopShooterMotorsCommand().schedule();
                ledSubsystem.idle();
        }

        public void disabledExit() {
                onInit();
        }

        public void disabledInit() {
                ledSubsystem.updateDisabledLEDs();
        }
}
