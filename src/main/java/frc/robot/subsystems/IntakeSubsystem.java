package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private final TalonFX intakeMotor;

    private final DigitalInput frontBeamBreak;
    private final DigitalInput backBeamBreak;

    public IntakeSubsystem() {
        // TODO: GET CAN IDs
        intakeMotor = new TalonFX(20);
        frontBeamBreak = new DigitalInput(3);
        backBeamBreak = new DigitalInput(2);

        intakeMotor.getPosition().setUpdateFrequency(4);

        applyConfigs();
    }

    private void applyConfigs() {
        TalonFXConfiguration INTAKE_MOTOR_CONFIG = new TalonFXConfiguration();
        INTAKE_MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        INTAKE_MOTOR_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // TODO: Change if needed
        
        // INTAKE_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        // INTAKE_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimit = 40;
        // INTAKE_MOTOR_CONFIG.CurrentLimits.SupplyCurrentThreshold = 50;
        // INTAKE_MOTOR_CONFIG.CurrentLimits.SupplyTimeThreshold = 1.0;

        intakeMotor.getConfigurator().apply(INTAKE_MOTOR_CONFIG);
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    // TODO: tune these values. they are in mm.
    public boolean isRingAtFrontOfIntake() {
        return !frontBeamBreak.get();
    }

    public boolean isRingFullyInsideIntake() {
        return !backBeamBreak.get();
    }

    public Command conveyIntoReadyPosition() {
        return this.run(() -> this.setIntakeSpeed(0.8)).until(() -> isRingFullyInsideIntake());
    }
}
