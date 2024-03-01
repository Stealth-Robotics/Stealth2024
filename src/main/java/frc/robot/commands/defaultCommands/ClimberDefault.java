package frc.robot.commands.defaultCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberDefault extends Command{

    private final ClimberSubsystem climber;
    private final DoubleSupplier climberManualControlSupplier;
    public ClimberDefault(DoubleSupplier climberManualControlSupplier, ClimberSubsystem climber){
        this.climber = climber;
        this.climberManualControlSupplier = climberManualControlSupplier;

        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.setPower(climberManualControlSupplier.getAsDouble());
    }
    
}
