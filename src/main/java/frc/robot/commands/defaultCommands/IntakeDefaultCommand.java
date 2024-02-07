package frc.robot.commands.defaultCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeDefaultCommand extends Command {

    private final IntakeSubsystem intake;
    private final DoubleSupplier bumper;

    public IntakeDefaultCommand(IntakeSubsystem intake, DoubleSupplier bumper) {
        this.bumper = bumper;
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setMotorSpeed(bumper.getAsDouble());
    }
    
}
