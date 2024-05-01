package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ClimberSubsystem extends SubsystemBase {
    private final TalonFX leftClimber;
    private final TalonFX rightClimber;

    TalonFXConfiguration CLIMBER_CONFIG = new TalonFXConfiguration();

    private final double kP = 0.0;
    private final double kI = 0.0;
    private final double kD = 0.0;
    private final double kTolerance = 0.0;

    private final PositionVoltage climberPositionVoltage = new PositionVoltage(0);

    public ClimberSubsystem() { // TODO: Get CAN IDs
        leftClimber = new TalonFX(18);
        rightClimber = new TalonFX(19);
        applyConfigs();
    }

    private void applyConfigs() {
        CLIMBER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        CLIMBER_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        CLIMBER_CONFIG.Slot0.kP = kP;
        CLIMBER_CONFIG.Slot0.kI = kI;
        CLIMBER_CONFIG.Slot0.kD = kD;

        rightClimber.getPosition().setUpdateFrequency(50);
        leftClimber.getPosition().setUpdateFrequency(50);

        rightClimber.getConfigurator().apply(CLIMBER_CONFIG);
        CLIMBER_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftClimber.getConfigurator().apply(CLIMBER_CONFIG);
    }

    private boolean getClimberAtSetpoint() {
        return (Math.abs(leftClimber.getPosition().getValueAsDouble() - climberPositionVoltage.Position) <= kTolerance
                && Math.abs(
                        rightClimber.getPosition().getValueAsDouble() - climberPositionVoltage.Position) <= kTolerance);
    }

    private void setTargetPosition(double position) {
        climberPositionVoltage.Position = position;
        leftClimber.setControl(climberPositionVoltage);
        rightClimber.setControl(climberPositionVoltage);
    }

    public Command climberToMaxHeight() {
        return new SequentialCommandGroup(
                // sets climber to 5 rotations of motor shaft, need to test
                new InstantCommand(() -> this.setTargetPosition(5), this),
                new WaitUntilCommand(this::getClimberAtSetpoint));
    }

    public void setLeftClimber(double power) {
        leftClimber.set(power);
    }

    public void setRightClimber(double power) {
        rightClimber.set(power);
    }
}
