package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class StowPreset extends ParallelCommandGroup {

    public StowPreset(RotatorSubsystem rotator, ShooterSubsystem shooter) {
        addCommands(rotator.rotateToPositionCommand(() -> Units.degreesToRadians(0)),
                shooter.stopShooterMotorsCommand());
    }
}
