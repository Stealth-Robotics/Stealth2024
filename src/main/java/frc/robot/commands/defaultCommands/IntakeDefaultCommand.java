package frc.robot.commands.defaultCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeDefaultCommand extends Command {

    private final IntakeSubsystem intake;
    private final DoubleSupplier intakeSupplier;

    public IntakeDefaultCommand(IntakeSubsystem intake, DoubleSupplier intakeSupplier) {
        this.intakeSupplier = intakeSupplier;
        this.intake = intake;
        addRequirements(intake);

        //TODO:Remove after command has been tested
        // throw new UnsupportedOperationException("IntakeDefaultCommand has not been tested on robot yet");
    }

    @Override
    public void execute() {
        if(intakeSupplier.getAsDouble() < 0){
            intake.setIntakeSpeed(intakeSupplier.getAsDouble());
            return;
        }
        if(intake.isRingFullyInsideIntake()){
            intake.setIntakeSpeed(0);
            return;
        }
        if(intake.isRingAtFrontOfIntake()){
            intake.setIntakeSpeed(0.3);
            return;
        }
        intake.setIntakeSpeed(intakeSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntakeSpeed(0);
    }
    
}
