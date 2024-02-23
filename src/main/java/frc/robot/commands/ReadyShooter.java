package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.shooter.DistanceToShotValuesMap;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;

public class ReadyShooter extends ParallelCommandGroup{
    public ReadyShooter(ShooterSubsystem shooter, RotatorSubsystem rotator, SwerveDrive drive) {
        double distance = drive.getDistanceMetersToGoal();
        addCommands(
            shooter.spinUpFromDistance(Units.metersToFeet(distance)),
            rotator.rotateToPositionCommand(DistanceToShotValuesMap.getInterpolatedRotationAngle(Units.metersToFeet(distance)))
        );
    }
    
}
