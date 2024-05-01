package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;

public class FollowPathAndReadyShooter extends ParallelCommandGroup {

    public FollowPathAndReadyShooter(SwerveDrive drive, IntakeSubsystem intakeSubsystem,
            RotatorSubsystem rotatorSubsystem, ShooterSubsystem shooterSubsystem, PathPlannerPath path,
            Translation2d distanceToShootFrom) {
        addCommands(drive.followPathCommand(path), new ReadyShooter(shooterSubsystem, rotatorSubsystem, intakeSubsystem,
                () -> drive.getDistanceMetersToGoal(distanceToShootFrom)));
    }
}
