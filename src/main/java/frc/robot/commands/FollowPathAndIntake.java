package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;

public class FollowPathAndIntake extends ParallelCommandGroup{

    public FollowPathAndIntake(SwerveDrive drive, IntakeSubsystem intakeSubsystem, RotatorSubsystem rotatorSubsystem, PathPlannerPath path, boolean isInitial){
        addCommands(
            drive.followPathCommand(path, isInitial),
            new SequentialCommandGroup(
                new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(1)),
                new WaitUntilCommand(() -> intakeSubsystem.isRingAtFrontOfIntake()).withTimeout(2.5),
                new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(0))
            ),
            rotatorSubsystem.rotateToPositionCommand(0)
        );
    }
    
}
