package frc.robot.commands;

import edu.wpi.first.math.util.Units;
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

public class Subwoofershot extends SequentialCommandGroup {

    public Subwoofershot(SwerveDrive drive, RotatorSubsystem rotator, ShooterSubsystem shooter,
            IntakeSubsystem intake, DistanceToShotValuesMap distanceToShotValuesMap, double distance) {
        addCommands(
                new InstantCommand(() -> intake.setIntakeSpeed(-0.2), intake)
                        .andThen(new WaitCommand(0.1))
                        .andThen(new InstantCommand(() -> intake.setIntakeSpeed(0), intake)),
                new ParallelCommandGroup(
                        rotator.rotateToPositionCommand(Units.degreesToRotations(27)),
                        shooter.spinToRps(90.0)
                                .withTimeout(1.5)),
                new RunCommand(() -> intake.setIntakeSpeed(1), intake)
                        .withTimeout(1.0),
                new InstantCommand(() -> intake.setIntakeSpeed(0)));
    }

}
