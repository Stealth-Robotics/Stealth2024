package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AimAndShootCommand;
import frc.robot.commands.ReadyShooter;
import frc.robot.commands.StowPreset;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.shooter.DistanceToShotValuesMap;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;

public class ThreeRingAmpSide extends SequentialCommandGroup {
        DistanceToShotValuesMap map = new DistanceToShotValuesMap();

        public ThreeRingAmpSide(SwerveDrive swerve, RotatorSubsystem rotator, ShooterSubsystem shooter,
                        IntakeSubsystem intake) {
                addCommands(
                                new InstantCommand(() -> swerve.setInitialPose("amp side left pickup")),
                                new ReadyShooter(shooter, rotator, intake, swerve, map),
                                new RunCommand(() -> intake.setIntakeSpeed(0.8), intake).withTimeout(0.5),
                                new InstantCommand(() -> intake.setIntakeSpeed(1)),
                                new ParallelCommandGroup(
                                                swerve.followPathCommand("amp side left pickup", true),
                                                new StowPreset(rotator, shooter)),
                                new WaitCommand(0.5),
                                swerve.followPathCommand("shoot amp ring", false),
                                new ReadyShooter(shooter, rotator, intake, swerve, map),
                                new RunCommand(() -> intake.setIntakeSpeed(0.8), intake).withTimeout(0.5),
                                new InstantCommand(() -> intake.setIntakeSpeed(0)),
                                new ParallelCommandGroup(
                                                swerve.followPathCommand("pickup middle ring 2", false),
                                                new InstantCommand(() -> intake.setIntakeSpeed(1)),
                                                new StowPreset(rotator, shooter)),
                                new WaitCommand(1),
                                new InstantCommand(() -> intake.setIntakeSpeed(0)),
                                new ParallelCommandGroup(
                                                swerve.followPathCommand("shoot middle ring 2", false),
                                                new ReadyShooter(shooter, rotator, intake, swerve, map,
                                                                swerve.getDistanceMetersToGoal(
                                                                                new Translation2d(2.487968842604352,
                                                                                6.02748285496571)))),
                                new RunCommand(() -> intake.setIntakeSpeed(0.8), intake).withTimeout(0.5),
                                new StowPreset(rotator, shooter),
                                new InstantCommand(() -> intake.setIntakeSpeed(0)));

        }

}
