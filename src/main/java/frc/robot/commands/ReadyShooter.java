package frc.robot.commands;

import edu.wpi.first.math.WPIMathJNI;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.shooter.DistanceToShotValuesMap;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;

public class ReadyShooter extends SequentialCommandGroup {

    ShooterSubsystem shooterSubsystem;
    RotatorSubsystem rotatorSubsystem;
    IntakeSubsystem intakeSubsystem;
    SwerveDrive swerveDrive;
    DistanceToShotValuesMap distanceToShotValuesMap;

    public ReadyShooter(ShooterSubsystem shooter, RotatorSubsystem rotator, IntakeSubsystem intake, SwerveDrive drive,
            DistanceToShotValuesMap distanceToShotValuesMap) {
        double distance = 4.0;

        double rotatorAngle = distanceToShotValuesMap.getInterpolatedRotationAngle(distance);
        double shooterRPS = distanceToShotValuesMap.getInterpolatedShooterSpeed(distance);

        addCommands(
                new InstantCommand(() -> intake.setIntakeSpeed(-0.2), intake).andThen(new WaitCommand(0.1))
                        .andThen(new InstantCommand(() -> intake.setIntakeSpeed(0), intake)),
                new WaitCommand(0.5),
                new ParallelCommandGroup(
                        // shooter.spinToRps(shooterRPS),
                        rotator.rotateToPositionCommand(rotatorAngle)));
                // new RunCommand(() -> intake.setIntakeSpeed(1), intake)
                        // .until(() -> !intake.isRingFullyInsideIntake()));
    }
}
