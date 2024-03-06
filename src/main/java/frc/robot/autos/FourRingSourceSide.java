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
                new RunCommand(() -> intake.setIntakeSpeed(0.8), intake).withTimeout(0.5),
                new InstantCommand(() -> intake.setIntakeSpeed(1)),
                new ParallelCommandGroup(
                        swerve.followPathCommand("right pickup first ring", true),
                        new StowPreset(rotator, shooter)),
                new WaitCommand(0.5),
                swerve.followPathCommand("shoot right first ring", false),
                new ReadyShooter(shooter, rotator, intake, swerve, map),
                new RunCommand(() -> intake.setIntakeSpeed(0.8), intake).withTimeout(0.5),
                new InstantCommand(() -> intake.setIntakeSpeed(0)),
                new ParallelCommandGroup(
                        swerve.followPathCommand("middle ring pickup", false),
                        new InstantCommand(() -> intake.setIntakeSpeed(1)),
                        new StowPreset(rotator, shooter)),
                new WaitCommand(0.5),
                new InstantCommand(() -> intake.setIntakeSpeed(0)),
                new ReadyShooter(shooter, rotator, intake, swerve, map),
                new RunCommand(() -> intake.setIntakeSpeed(0.8), intake).withTimeout(0.5),
                new InstantCommand(() -> intake.setIntakeSpeed(0)),
                new ParallelCommandGroup(
                        swerve.followPathCommand("left ring pickup", false),
                        new InstantCommand(() -> intake.setIntakeSpeed(1)),
                        new StowPreset(rotator, shooter)),
                new WaitCommand(0.5),
                new InstantCommand(() -> intake.setIntakeSpeed(0)),
                swerve.followPathCommand("shoot left ring", false),
                new ReadyShooter(shooter, rotator, intake, swerve, map),
                new RunCommand(() -> intake.setIntakeSpeed(0.8), intake).withTimeout(0.5),
                new InstantCommand(() -> intake.setIntakeSpeed(0)),
                new StowPreset(rotator, shooter)

        );
    }

}
