package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    private final TalonFX leftClimber;
    private final TalonFX rightClimber;

    TalonFXConfiguration CLIMBER_CONFIG = new TalonFXConfiguration();

    public ClimberSubsystem() {
        leftClimber = new TalonFX(18);
        rightClimber = new TalonFX(19);
        applyConfigs();
    }

    private void applyConfigs() {
        CLIMBER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        CLIMBER_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        rightClimber.getPosition().setUpdateFrequency(4);
        leftClimber.getPosition().setUpdateFrequency(4);

        rightClimber.getConfigurator().apply(CLIMBER_CONFIG);
        CLIMBER_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftClimber.getConfigurator().apply(CLIMBER_CONFIG);

    }

    public void setPower(double power) {
        leftClimber.set(power);
        rightClimber.set(power);
    }
}
