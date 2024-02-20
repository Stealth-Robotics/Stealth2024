package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;

public class AimAndShootCommand extends SequentialCommandGroup{

    public AimAndShootCommand(SwerveDrive drive, ShooterSubsystem shooter, IntakeSubsystem intake){
        addRequirements(drive, intake, shooter);

        addCommands(
            new ParallelCommandGroup(
                new AutoAlignCommand(drive),
                new ReadyShooterCommand(shooter, drive)
            ),
            new RunCommand(() -> intake.setIntakeSpeed(0.4), intake).until(() -> !intake.isRingFullyInsideIntake())
        );
    }
    
}
