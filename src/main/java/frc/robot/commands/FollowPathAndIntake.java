package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;

public class FollowPathAndIntake extends ParallelCommandGroup {

    public FollowPathAndIntake(SwerveDrive drive, IntakeSubsystem intakeSubsystem, RotatorSubsystem rotatorSubsystem,
            PathPlannerPath path) {
        addCommands(drive.followPathCommand(path),
                new SequentialCommandGroup(new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(1), intakeSubsystem),
                        new WaitUntilCommand(() -> intakeSubsystem.isRingAtFrontOfIntake()).withTimeout(2.5),
                        new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(0), intakeSubsystem)),
                rotatorSubsystem.rotateToPositionCommand(() -> Units.degreesToRotations(0)));
    }
}
