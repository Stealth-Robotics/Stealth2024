package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ReadyShooter;
import frc.robot.commands.StowPreset;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;

public class TwoRingSourceSide extends SequentialCommandGroup {

        public TwoRingSourceSide(SwerveDrive swerve, RotatorSubsystem rotator, ShooterSubsystem shooter,
                        IntakeSubsystem intake) {
                addCommands(new InstantCommand(() -> swerve.setInitialPose("subwoofer to middle ring 4 pickup")),
                                new ReadyShooter(shooter, rotator, intake, () -> swerve.getDistanceMetersToGoal()),
                                new RunCommand(() -> intake.setIntakeSpeed(0.8), intake).withTimeout(0.5),
                                new InstantCommand(() -> intake.setIntakeSpeed(1)), new StowPreset(rotator, shooter),
                                new WaitCommand(0.5),
                                new ParallelCommandGroup(swerve.followPathCommand("subwoofer to middle ring 4 pickup"),
                                                new InstantCommand(() -> intake.setIntakeSpeed(1))),
                                new WaitCommand(0.5), new InstantCommand(() -> intake.setIntakeSpeed(0)),
                                new ParallelCommandGroup(swerve.followPathCommand("middle to shoot variation")),
                                new ReadyShooter(shooter, rotator, intake, () -> swerve.getDistanceMetersToGoal()),
                                new RunCommand(() -> intake.setIntakeSpeed(0.8), intake).withTimeout(0.5),
                                new StowPreset(rotator, shooter), new InstantCommand(() -> intake.setIntakeSpeed(0)));
        }
}
