package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.WPIMathJNI;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
        private double distance = 0;
        private double angle = 0;
        private double speed = 0;

        public ReadyShooter(ShooterSubsystem shooter, RotatorSubsystem rotator, IntakeSubsystem intake,
                        SwerveDrive drive,
                        DistanceToShotValuesMap distanceToShotValuesMap) {
                this.swerveDrive = drive;

                this.intake = intake;
                this.rotator = rotator;
                this.shooter = shooter;
                addCommands(
                                new InstantCommand(() -> {double meters = drive.getDistanceMetersToGoal(); System.out.println(meters);}),
                                new InstantCommand(() -> intake.setIntakeSpeed(-0.2), intake)
                                                .andThen(new WaitCommand(0.1))
                                                .andThen(new InstantCommand(() -> intake.setIntakeSpeed(0), intake)),
                                new WaitCommand(0.1),
                                new SubsystemsToTarget(rotator, shooter, drive, distanceToShotValuesMap));

        }

        public ReadyShooter(ShooterSubsystem shooter, RotatorSubsystem rotator, IntakeSubsystem intake,
                        SwerveDrive drive,
                        DistanceToShotValuesMap distanceToShotValuesMap, double distance) {
                this.shooter = shooter;
                this.rotator = rotator;
                this.intake = intake;
                this.swerveDrive = drive;
                this.distanceToShotValuesMap = distanceToShotValuesMap;
                this.distance = distance;
                addCommands(
                                new InstantCommand(() -> intake.setIntakeSpeed(-0.2), intake)
                                                .andThen(new WaitCommand(0.1))
                                                .andThen(new InstantCommand(() -> intake.setIntakeSpeed(0), intake)),
                                new WaitCommand(0.1),
                                new SubsystemsToTarget(rotator, shooter, drive, distanceToShotValuesMap, distance));

        }


        public ReadyShooter(ShooterSubsystem shooter, RotatorSubsystem rotator, IntakeSubsystem intake,
                        SwerveDrive drive,
                        DistanceToShotValuesMap distanceToShotValuesMap, DoubleSupplier distanceOffset) {
                this.swerveDrive = drive;

                this.intake = intake;
                this.rotator = rotator;
                this.shooter = shooter;
                addCommands(
                                new InstantCommand(() -> intake.setIntakeSpeed(-0.2), intake)
                                                .andThen(new WaitCommand(0.1))
                                                .andThen(new InstantCommand(() -> intake.setIntakeSpeed(0), intake)),
                                new WaitCommand(0.1),
                                new SubsystemsToTarget(rotator, shooter, drive, distanceToShotValuesMap));

        }
}
