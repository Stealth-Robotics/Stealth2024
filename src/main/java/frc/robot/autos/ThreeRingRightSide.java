package frc.robot.autos;

import com.pathplanner.lib.path.PathPlannerPath;

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

public class ThreeRingRightSide extends SequentialCommandGroup {
        DistanceToShotValuesMap map = new DistanceToShotValuesMap();

        public ThreeRingRightSide(SwerveDrive swerve, RotatorSubsystem rotator, ShooterSubsystem shooter,
                        IntakeSubsystem intake) {
                addCommands(
                                new InstantCommand(() -> swerve.setInitialPose("right pickup first ring")),
                                new ReadyShooter(shooter, rotator, intake, swerve, map).withTimeout(1.5),
                                new RunCommand(() -> intake.setIntakeSpeed(1), intake).withTimeout(0.5),
                                new InstantCommand(() -> intake.setIntakeSpeed(1)),
                                new ParallelCommandGroup(
                                                swerve.followPathCommand(PathPlannerPath
                                                                .fromChoreoTrajectory("pickup ring source side"), true),
                                                new StowPreset(rotator, shooter)),
                                new WaitCommand(0.1),
                                swerve.followPathCommand(
                                                PathPlannerPath.fromChoreoTrajectory("shoot ring 1 source side"),
                                                false),
                                new ReadyShooter(shooter, rotator, intake, swerve, map).withTimeout(1.5),
                                new RunCommand(() -> intake.setIntakeSpeed(1), intake).withTimeout(0.5),
                                new InstantCommand(() -> intake.setIntakeSpeed(0)),
                                new ParallelCommandGroup(
                                                swerve.followPathCommand(PathPlannerPath
                                                                .fromChoreoTrajectory("pickup middle ring 4"), false),
                                                new InstantCommand(() -> intake.setIntakeSpeed(1)),
                                                new StowPreset(rotator, shooter)),
                                new WaitCommand(0.25),
                                new InstantCommand(() -> intake.setIntakeSpeed(0)),
                                new ParallelCommandGroup(
                                                swerve.followPathCommand(PathPlannerPath.fromChoreoTrajectory(
                                                                "drive to shoot middle ring 4"), false),
                                                new ReadyShooter(shooter, rotator, intake, swerve, map,
                                                                swerve.getDistanceMetersToGoal(
                                                                                new Translation2d(1.8611713162512615,
                                                                                                4.525530919082143)))
                                                                .withTimeout(2)),
                                new WaitCommand(0.5),
                                new RunCommand(() -> intake.setIntakeSpeed(0.8), intake).withTimeout(0.5),
                                new StowPreset(rotator, shooter),
                                new InstantCommand(() -> intake.setIntakeSpeed(0)));

        }

}
