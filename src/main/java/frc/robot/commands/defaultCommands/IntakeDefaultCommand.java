package frc.robot.commands.defaultCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeDefaultCommand extends Command {

    private final IntakeSubsystem intake;

    private final DoubleSupplier intakeSupplier;

    public IntakeDefaultCommand(IntakeSubsystem intake, DoubleSupplier intakeSupplier) {
        this.intakeSupplier = intakeSupplier;

        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        if(intakeSupplier.getAsDouble() < 0){
            intake.setIntakeSpeed(intakeSupplier.getAsDouble());
            return;
        }
        if (intake.isRingAtFrontOfIntake()) {
            intake.setIntakeSpeed(0.7);
        }
        if(intake.isRingFullyInsideIntake()){
            intake.setIntakeSpeed(0);
            return;
        }
        intake.setIntakeSpeed(intakeSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntakeSpeed(0);
    }

}
