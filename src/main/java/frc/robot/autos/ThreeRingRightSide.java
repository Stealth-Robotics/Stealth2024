package frc.robot.autos;

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
                new ReadyShooter(shooter, rotator, intake, swerve, map),
                new RunCommand(() -> intake.setIntakeSpeed(0.8), intake).withTimeout(0.5),
                new InstantCommand(() -> intake.setIntakeSpeed(1)),
                new ParallelCommandGroup(
                        swerve.followPathCommand("right pickup first ring", true),
                        new StowPreset(rotator, shooter)),
                new WaitCommand(1),
                new ReadyShooter(shooter, rotator, intake, swerve, map),
                new RunCommand(() -> intake.setIntakeSpeed(0.8), intake).withTimeout(0.5),
                new InstantCommand(() -> intake.setIntakeSpeed(0)),
                new ParallelCommandGroup(
                        swerve.followPathCommand("pickup middle ring 4", false),
                        new InstantCommand(() -> intake.setIntakeSpeed(1)),
                        new StowPreset(rotator, shooter)),
                new WaitCommand(1),
                new InstantCommand(() -> intake.setIntakeSpeed(0)),
                swerve.followPathCommand("middle to shoot variation", false),
                new ReadyShooter(shooter, rotator, intake, swerve, map),
                new RunCommand(() -> intake.setIntakeSpeed(0.8), intake).withTimeout(0.5),
                new StowPreset(rotator, shooter),
                new InstantCommand(() -> intake.setIntakeSpeed(0)));

    }

}
