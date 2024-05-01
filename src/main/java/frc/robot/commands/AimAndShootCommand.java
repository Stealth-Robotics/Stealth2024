package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.util.function.DoubleSupplier;

public class AimAndShootCommand extends SequentialCommandGroup {

    public AimAndShootCommand(SwerveDrive drive, RotatorSubsystem rotator, ShooterSubsystem shooter,
            IntakeSubsystem intake, DoubleSupplier distanceFromGoalSupplier) {
        addCommands(
                new ParallelCommandGroup(new ReadyShooter(shooter, rotator, intake, distanceFromGoalSupplier),
                        new AutoAlignCommand(drive)),
                new RunCommand(() -> intake.setIntakeSpeed(1), intake).withTimeout(1.5));
    }
}
