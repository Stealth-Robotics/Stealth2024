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

public class CenterTwoRing extends SequentialCommandGroup {
        DistanceToShotValuesMap map = new DistanceToShotValuesMap();

        public CenterTwoRing(SwerveDrive swerve, RotatorSubsystem rotator, ShooterSubsystem shooter,
                        IntakeSubsystem intake) {
                addCommands(
                                new InstantCommand(() -> swerve.setInitialPose("center first pickup")),
                                new ReadyShooter(shooter, rotator, intake, swerve, map, 1.4),
                                new RunCommand(() -> intake.setIntakeSpeed(0.8), intake).withTimeout(0.75),
                                new InstantCommand(() -> intake.setIntakeSpeed(1)),
                                new StowPreset(rotator, shooter),
                                swerve.followPathCommand("center first pickup", true),

                                new WaitCommand(0.5),
                                new InstantCommand(() -> intake.setIntakeSpeed(0)),
                                swerve.followPathCommand("ring to subwoofer shoot", false),
                                new ReadyShooter(shooter, rotator, intake, swerve, map, 1.4),
                                new RunCommand(() -> intake.setIntakeSpeed(0.8), intake).withTimeout(0.5),
                                new InstantCommand(() -> intake.setIntakeSpeed(0)),
                                new StowPreset(rotator, shooter),
                                new InstantCommand(() -> intake.setIntakeSpeed(0)));

        }

}
