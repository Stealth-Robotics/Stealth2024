package frc.robot.autos;

import java.nio.file.Path;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.ReadyShooter;
import frc.robot.commands.StowPreset;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.shooter.DistanceToShotValuesMap;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;

public class FourRingSourceSide extends SequentialCommandGroup {
        DistanceToShotValuesMap map = new DistanceToShotValuesMap();

        public FourRingSourceSide(SwerveDrive swerve, RotatorSubsystem rotator, ShooterSubsystem shooter,
                        IntakeSubsystem intake) {
                addCommands(
                                new InstantCommand(() -> swerve.setInitialPose("right pickup first ring")),
                                new ReadyShooter(shooter, rotator, intake, swerve, map),
                                new RunCommand(() -> intake.setIntakeSpeed(1), intake).withTimeout(0.5),
                                new InstantCommand(() -> intake.setIntakeSpeed(1)),
                                new ParallelCommandGroup(
                                                swerve.followPathCommand(PathPlannerPath
                                                                .fromChoreoTrajectory("pickup ring source side"), true),
                                                rotator.rotateToPositionCommand(0)),
                                new SequentialCommandGroup(
                                                new WaitUntilCommand(() -> intake.isRingAtFrontOfIntake())
                                                                .withTimeout(2.5),
                                                new InstantCommand(() -> intake.setIntakeSpeed(0))),
                                new WaitCommand(0.25),
                                new ParallelCommandGroup(
                                                swerve.followPathCommand(PathPlannerPath.fromChoreoTrajectory(
                                                                "shoot ring 1 before pickup"), false),
                                                new ReadyShooter(shooter, rotator, intake, swerve, map, swerve
                                                                .getDistanceMetersToGoal(
                                                                                new Translation2d(1.772, 4.909)))),
                                new RunCommand(() -> intake.setIntakeSpeed(1), intake).withTimeout(0.5),
                                new InstantCommand(() -> intake.setIntakeSpeed(0)),
                                new ParallelCommandGroup(
                                                swerve.followPathCommand(
                                                                PathPlannerPath.fromChoreoTrajectory("pickup ring 2"),
                                                                false),
                                                new InstantCommand(() -> intake.setIntakeSpeed(1)),
                                                new SequentialCommandGroup(
                                                                new WaitUntilCommand(
                                                                                () -> intake.isRingAtFrontOfIntake())
                                                                                .withTimeout(2.5),
                                                                new InstantCommand(() -> intake.setIntakeSpeed(0))),
                                                rotator.rotateToPositionCommand(0)),
                                new WaitCommand(0.25),
                                new InstantCommand(() -> intake.setIntakeSpeed(0)),
                                new ReadyShooter(shooter, rotator, intake, swerve, map),
                                new RunCommand(() -> intake.setIntakeSpeed(1), intake).withTimeout(0.5),
                                new InstantCommand(() -> intake.setIntakeSpeed(0)),
                                new ParallelCommandGroup(
                                                swerve.followPathCommand(
                                                                PathPlannerPath.fromChoreoTrajectory("pickup ring 3"),
                                                                false),
                                                new InstantCommand(() -> intake.setIntakeSpeed(1)),
                                                rotator.rotateToPositionCommand(0)),
                                new SequentialCommandGroup(
                                                new WaitUntilCommand(() -> intake.isRingAtFrontOfIntake())
                                                                .withTimeout(2.5),
                                                new InstantCommand(() -> intake.setIntakeSpeed(0))),
                                new WaitCommand(0.5),
                                new InstantCommand(() -> intake.setIntakeSpeed(0)),
                                new ParallelCommandGroup(
                                                swerve.followPathCommand(
                                                                PathPlannerPath.fromChoreoTrajectory("shoot ring 3"),
                                                                false),
                                                new ReadyShooter(shooter, rotator, intake, swerve, map,
                                                                swerve.getDistanceMetersToGoal(new Translation2d(
                                                                                2.358, 6.213)))),
                                new RunCommand(() -> intake.setIntakeSpeed(1), intake).withTimeout(0.5),
                                new InstantCommand(() -> intake.setIntakeSpeed(0)),
                                new ParallelCommandGroup(
                                                rotator.rotateToPositionCommand(0),
                                                swerve.followPathCommand(PathPlannerPath
                                                                .fromChoreoTrajectory("pickup middle ring 1"), false),
                                                new InstantCommand(() -> intake.setIntakeSpeed(1))),
                                new ParallelCommandGroup(
                                                swerve.followPathCommand(PathPlannerPath
                                                                .fromChoreoTrajectory("shoot middle ring 1"), false),
                                                new ReadyShooter(shooter, rotator, intake, swerve, map,
                                                                swerve.getDistanceMetersToGoal(
                                                                                new Translation2d(3.843, 6.508)))),
                                new RunCommand(() -> intake.setIntakeSpeed(1), intake).withTimeout(0.5),
                                new InstantCommand(() -> intake.setIntakeSpeed(0)),
                                new StowPreset(rotator, shooter)

                );
        }

}
