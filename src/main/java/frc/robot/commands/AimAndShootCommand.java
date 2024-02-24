package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;

public class AimAndShootCommand extends SequentialCommandGroup {

    public AimAndShootCommand(SwerveDrive drive, RotatorSubsystem rotator, ShooterSubsystem shooter,
            IntakeSubsystem intake) {
        addCommands(
                new ParallelCommandGroup(
                        new AutoAlignCommand(drive),
                        new ReadyShooter(shooter, rotator, drive)),
                new RunCommand(() -> intake.setIntakeSpeed(0.8), intake).until(() -> !intake.isRingFullyInsideIntake()),
                new WaitCommand(1),
                new InstantCommand(() -> intake.setIntakeSpeed(0)));
    }

}
