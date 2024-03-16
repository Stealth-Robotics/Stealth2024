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
import frc.robot.commands.FollowPathAndIntake;
import frc.robot.commands.FollowPathAndReadyShooter;
import frc.robot.commands.ReadyShooter;
import frc.robot.commands.StowPreset;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.shooter.DistanceToShotValuesMap;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;

public class CenterFiveRing extends SequentialCommandGroup {
        DistanceToShotValuesMap map = new DistanceToShotValuesMap();

        public CenterFiveRing(SwerveDrive swerve, RotatorSubsystem rotator, ShooterSubsystem shooter,
                        IntakeSubsystem intake) {
                addCommands(
                                new InstantCommand(() -> swerve.setInitialPose("right pickup first ring")),
                                new ReadyShooter(shooter, rotator, intake, swerve, map),
                                new RunCommand(() -> intake.setIntakeSpeed(1), intake).withTimeout(0.5),
                                new InstantCommand(() -> intake.setIntakeSpeed(1)),
                                new FollowPathAndIntake(swerve, intake, rotator,
                                                PathPlannerPath.fromPathFile("right pickup first ring"), true),
                                new FollowPathAndReadyShooter(swerve, intake, rotator, shooter, map,
                                                PathPlannerPath.fromPathFile("shoot right first ring"),
                                                isScheduled(), new Translation2d(1.70, 4.22)),
                                new RunCommand(() -> intake.setIntakeSpeed(1), intake).withTimeout(0.5),
                                new InstantCommand(() -> intake.setIntakeSpeed(0)),
                                new FollowPathAndIntake(swerve, intake, rotator,
                                                PathPlannerPath.fromPathFile("middle ring pickup"), false),
                                new ReadyShooter(shooter, rotator, intake, swerve, map),
                                new RunCommand(() -> intake.setIntakeSpeed(1), intake).withTimeout(0.5),
                                new InstantCommand(() -> intake.setIntakeSpeed(0)),
                                new FollowPathAndIntake(swerve, intake, rotator,
                                                PathPlannerPath.fromPathFile("left ring pickup"), false),
                                new FollowPathAndReadyShooter(swerve, intake, rotator, shooter, map,
                                                PathPlannerPath.fromPathFile("shoot left ring"), isScheduled(),
                                                new Translation2d(
                                                                1.81, 6.06)),
                                new RunCommand(() -> intake.setIntakeSpeed(1), intake).withTimeout(0.5),
                                new InstantCommand(() -> intake.setIntakeSpeed(0)),
                                new StowPreset(rotator, shooter),
                                new InstantCommand(() -> intake.setIntakeSpeed(1)),
                                swerve.followPathCommand("pickup middle ring 2", false),
                                new ParallelCommandGroup(
                                                new SequentialCommandGroup(
                                                                new WaitCommand(0.5),
                                                                new InstantCommand(() -> intake.setIntakeSpeed(0))),
                                                new FollowPathAndReadyShooter(swerve, intake, rotator, shooter, map,
                                                                PathPlannerPath.fromPathFile("shoot middle ring 2"),
                                                                false,
                                                                new Translation2d(2.49, 6.03))),
                                new RunCommand(() -> intake.setIntakeSpeed(1), intake).withTimeout(0.5),
                                new InstantCommand(() -> intake.setIntakeSpeed(0)),
                                new StowPreset(rotator, shooter)

                );
        }

}
