package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.shooter.DistanceToShotValuesMap;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.function.DoubleSupplier;

public class ReadyShooter extends SequentialCommandGroup {
    DistanceToShotValuesMap distanceToShotValuesMap = new DistanceToShotValuesMap();

    public ReadyShooter(ShooterSubsystem shooter, RotatorSubsystem rotator, IntakeSubsystem intake,
            DoubleSupplier distanceFromGoalSupplier) {
        addCommands(new InstantCommand(() -> intake.setIntakeSpeed(-0.2), intake)
                .andThen(new WaitCommand(0.1)).andThen(new InstantCommand(() -> intake.setIntakeSpeed(0), intake)),
                new WaitCommand(0.1),
                new ParallelCommandGroup(
                        rotator.rotateToPositionCommand(() -> (distanceToShotValuesMap
                                .getInterpolatedRotationAngle(distanceFromGoalSupplier.getAsDouble()))),
                        shooter.spinToRps(() -> distanceToShotValuesMap
                                .getInterpolatedShooterSpeed(distanceFromGoalSupplier.getAsDouble()))));
    }
}
