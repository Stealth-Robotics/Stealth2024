package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class AmpPresetCommand extends ParallelCommandGroup{

    public AmpPresetCommand(RotatorSubsystem rotator, ShooterSubsystem shooter){
        addCommands(
            rotator.rotateToPositionCommand(Units.degreesToRotations(52))
            
        );
    }
    
}
