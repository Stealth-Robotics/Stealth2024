package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class StowPreset extends SequentialCommandGroup{

    public StowPreset(RotatorSubsystem rotator, ShooterSubsystem shooter){
        addCommands(
            rotator.rotateToPositionCommand(Units.degreesToRotations(0)),
            new InstantCommand(() -> shooter.stopShooterMotors()),
            new InstantCommand(() -> rotator.setDutyCycle(0))
        );
    }
    
}
