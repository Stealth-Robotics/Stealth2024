package frc.robot.autos;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.commands.FollowPathAndReadyShooter;
import frc.robot.commands.ReadyShooter;
import frc.robot.commands.StowPreset;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RotatorSubsystem;

import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;

public class ThreeRingRightSide extends SequentialCommandGroup {


        public ThreeRingRightSide(SwerveDrive swerve, RotatorSubsystem rotator, ShooterSubsystem shooter,
                        IntakeSubsystem intake) {
                addCommands(
                                new InstantCommand(() -> swerve.setInitialPose("right ring")),
                                new ReadyShooter(shooter, rotator, intake, 
                                                () -> swerve.getDistanceMetersToGoal()).withTimeout(2),
                                new RunCommand(() -> intake.setIntakeSpeed(1), intake).withTimeout(0.5),
                                new InstantCommand(() -> intake.setIntakeSpeed(1)),
                                new ParallelCommandGroup(
                                                swerve.followPathCommand(PathPlannerPath
                                                                .fromPathFile("right ring")),
                                                new StowPreset(rotator, shooter)),
                                new WaitCommand(0.1),
                                swerve.followPathCommand(
                                                PathPlannerPath.fromPathFile("shoot right first ring")),
                                new ReadyShooter(shooter, rotator, intake, 
                                                () -> swerve.getDistanceMetersToGoal()).withTimeout(2),
                                new RunCommand(() -> intake.setIntakeSpeed(1), intake).withTimeout(0.5),
                                new InstantCommand(() -> intake.setIntakeSpeed(0)),
                                new ParallelCommandGroup(
                                                swerve.followPathCommand(PathPlannerPath
                                                                .fromPathFile("pickup middle ring 4")),
                                                new InstantCommand(() -> intake.setIntakeSpeed(1)),
                                                new StowPreset(rotator, shooter)),
                                new WaitCommand(0.25),
                                new InstantCommand(() -> intake.setIntakeSpeed(0)),
                                new FollowPathAndReadyShooter(swerve, intake, rotator, shooter,
                                                PathPlannerPath.fromPathFile("middle to shoot variation"),
                                                new Translation2d(1.92, 4.28)),

                                new WaitCommand(0.5),
                                new RunCommand(() -> intake.setIntakeSpeed(0.8), intake).withTimeout(0.5),
                                new StowPreset(rotator, shooter),
                                new InstantCommand(() -> intake.setIntakeSpeed(0)));

        }

}
