package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.shooter.DistanceToShotValuesMap;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;

public class AimAndShootCommand extends SequentialCommandGroup {

    public AimAndShootCommand(SwerveDrive drive, RotatorSubsystem rotator, ShooterSubsystem shooter, IntakeSubsystem intake, DistanceToShotValuesMap distanceToShotValuesMap) {
        addCommands(
                new ParallelCommandGroup(
                        new AutoAlignCommand(drive).withTimeout(2),
                        new ReadyShooter(shooter, rotator, intake, drive, distanceToShotValuesMap)).withTimeout(2),
                        new RunCommand(() -> intake.setIntakeSpeed(1), intake)
                                                .until(() -> !intake.isRingFullyInsideIntake())
                        );
    }

    public AimAndShootCommand(SwerveDrive drive, RotatorSubsystem rotator, ShooterSubsystem shooter, IntakeSubsystem intake, DistanceToShotValuesMap distanceToShotValuesMap, double distance) {
        addCommands(
                new ParallelCommandGroup(
                        new ReadyShooter(shooter, rotator, intake, drive, distanceToShotValuesMap, distance)).withTimeout(1.5),
                        new RunCommand(() -> intake.setIntakeSpeed(1), intake)
                                                .withTimeout(1.0)
                        );
    }

}
