package frc.robot.commands.defaultCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberDefault extends Command {

    private final ClimberSubsystem climber;
    private final DoubleSupplier climberLeftManualControlSupplier;
    private final DoubleSupplier climberRightManualControlSupplier;

    public ClimberDefault(DoubleSupplier climberLeftManualControlSupplier,
            DoubleSupplier climberRightManualControlSupplier, ClimberSubsystem climber) {
        this.climber = climber;
        this.climberLeftManualControlSupplier = climberLeftManualControlSupplier;
        this.climberRightManualControlSupplier = climberRightManualControlSupplier;

        addRequirements(climber);
    }

    @Override
    public void execute() {
        if (Math.abs(climberLeftManualControlSupplier.getAsDouble()) > 0.5) {
            climber.setLeftClimber(-climberLeftManualControlSupplier.getAsDouble());
        }

        else {
            climber.setLeftClimber(0);
        }

        if (Math.abs(climberRightManualControlSupplier.getAsDouble()) > 0.5) {
            climber.setRightClimber(-climberRightManualControlSupplier.getAsDouble());
        }

        else {
            climber.setRightClimber(0);
        }
    }

}
