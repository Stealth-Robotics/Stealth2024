package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class AmpPresetCommand extends ParallelCommandGroup {

    public AmpPresetCommand(RotatorSubsystem rotator, ShooterSubsystem shooter, IntakeSubsystem intake) {
        addCommands(
                new InstantCommand(() -> intake.setIntakeSpeed(-0.2), intake)
                        .andThen(new WaitCommand(0.1))
                        .andThen(new InstantCommand(() -> intake.setIntakeSpeed(0), intake)),
                new WaitCommand(0.1),
                rotator.rotateToPositionCommand(() -> Units.degreesToRotations(101)),
                shooter.spinToRps(() -> 35)

        );
    }

}
