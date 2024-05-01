package frc.robot.autos;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FollowPathAndIntake;
import frc.robot.commands.FollowPathAndReadyShooter;
import frc.robot.commands.ReadyShooter;
import frc.robot.commands.StowPreset;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;

public class CenterAuto extends SequentialCommandGroup {

    public CenterAuto(SwerveDrive swerve, RotatorSubsystem rotator, ShooterSubsystem shooter, IntakeSubsystem intake) {
        addCommands(
                new InstantCommand(
                        () -> swerve.setInitialPose(PathPlannerPath.fromChoreoTrajectory("center pickup ring 1"))),
                new ReadyShooter(shooter, rotator, intake, () -> swerve.getDistanceMetersToGoal()),
                new RunCommand(() -> intake.setIntakeSpeed(1), intake).withTimeout(0.5),
                new FollowPathAndIntake(swerve, intake, rotator,
                        PathPlannerPath.fromChoreoTrajectory("center pickup ring 1")),
                new ReadyShooter(shooter, rotator, intake, () -> swerve.getDistanceMetersToGoal()),
                new RunCommand(() -> intake.setIntakeSpeed(1), intake).withTimeout(0.5),
                new FollowPathAndIntake(swerve, intake, rotator, PathPlannerPath.fromChoreoTrajectory("pickup ring 3")),
                new FollowPathAndReadyShooter(swerve, intake, rotator, shooter,
                        PathPlannerPath.fromChoreoTrajectory("shoot ring 3"), new Translation2d(2.358, 6.213)),
                new RunCommand(() -> intake.setIntakeSpeed(1), intake).withTimeout(0.5),
                new InstantCommand(() -> intake.setIntakeSpeed(0)),
                new ParallelCommandGroup(
                        new FollowPathAndIntake(swerve, intake, rotator,
                                PathPlannerPath.fromChoreoTrajectory("pickup middle ring 1")),
                        shooter.stopShooterMotorsCommand()),
                new FollowPathAndReadyShooter(swerve, intake, rotator, shooter,
                        PathPlannerPath.fromChoreoTrajectory("shoot middle ring 1"),
                        new Translation2d(2.6544923782348633, 6.280165195465088)),
                new RunCommand(() -> intake.setIntakeSpeed(1), intake).withTimeout(0.5),
                new InstantCommand(() -> intake.setIntakeSpeed(0)), new StowPreset(rotator, shooter));
    }
}
