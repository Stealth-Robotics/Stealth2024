package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class StowPreset extends SequentialCommandGroup{

    public StowPreset(RotatorSubsystem rotator, ShooterSubsystem shooter){
        addCommands(
            rotator.rotateToPositionCommand(0),
            shooter.stopShooterMotors()
        );
    }
    
}
