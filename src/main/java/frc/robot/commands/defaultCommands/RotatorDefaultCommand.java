package frc.robot.commands.defaultCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RotatorSubsystem;

public class RotatorDefaultCommand extends Command {

    private final RotatorSubsystem rotatorSubsystem;

    private final DoubleSupplier rotatorDutyCycle;

    private boolean setOnce = false;

    public RotatorDefaultCommand(RotatorSubsystem rotatorSubsystem, DoubleSupplier rotatorDutyCycle) {

        this.rotatorSubsystem = rotatorSubsystem;
        this.rotatorDutyCycle = rotatorDutyCycle;

        addRequirements(rotatorSubsystem);
    }

    @Override
    public void execute() {
        if(Math.abs(rotatorDutyCycle.getAsDouble()) > 0.1) {
            rotatorSubsystem.setDutyCycle(rotatorDutyCycle.getAsDouble());
            setOnce = false;
        }

        else if (!setOnce) {
            rotatorSubsystem.holdCurrentPosition();
            setOnce = true;
        }
    }    
    
}
