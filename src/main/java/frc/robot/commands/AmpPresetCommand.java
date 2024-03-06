package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class AmpPresetCommand extends ParallelCommandGroup{

    public AmpPresetCommand(RotatorSubsystem rotator, ShooterSubsystem shooter){
        addCommands(
            rotator.rotateToPositionCommand(Units.degreesToRotations(101)),
            shooter.spinToRps(35)
            
        );
    }
    
}
