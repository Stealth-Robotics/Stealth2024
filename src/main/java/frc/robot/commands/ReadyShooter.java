package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.shooter.DistanceToShotValuesMap;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;

public class ReadyShooter extends SequentialCommandGroup {

        ShooterSubsystem shooter;
        RotatorSubsystem rotator;
        IntakeSubsystem intake;
        SwerveDrive swerveDrive;
        DistanceToShotValuesMap distanceToShotValuesMap;

        public ReadyShooter(ShooterSubsystem shooter, RotatorSubsystem rotator, IntakeSubsystem intake,
                        SwerveDrive drive,
                        DistanceToShotValuesMap distanceToShotValuesMap) {
                this.swerveDrive = drive;

                this.intake = intake;
                this.rotator = rotator;
                this.shooter = shooter;
                addCommands(
                                new InstantCommand(() -> {
                                        double meters = drive.getDistanceMetersToGoal();
                                        System.out.println(meters);
                                }),
                                new InstantCommand(() -> intake.setIntakeSpeed(-0.2), intake)
                                                .andThen(new WaitCommand(0.1))
                                                .andThen(new InstantCommand(() -> intake.setIntakeSpeed(0), intake)),
                                new WaitCommand(0.5),
                                new SubsystemsToTarget(rotator, shooter, drive, distanceToShotValuesMap));

        }
}
