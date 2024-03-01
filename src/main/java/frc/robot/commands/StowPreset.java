package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class StowPreset extends ParallelCommandGroup{

    public StowPreset(RotatorSubsystem rotator, ShooterSubsystem shooterSubsystem){
        addCommands(
            rotator.rotateToPositionCommand(0),
            new InstantCommand(() -> shooterSubsystem.stopShooterMotors())
        );
    }
    
}
