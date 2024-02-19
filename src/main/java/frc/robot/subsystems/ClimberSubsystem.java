package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem {
    private final TalonFX leftClimber;
    private final TalonFX rightClimber;

    TalonFXConfiguration LEFT_TALONFX_CONFIG = new TalonFXConfiguration();
    TalonFXConfiguration RIGHT_TALONFX_CONFIG = new TalonFXConfiguration();
    
    public ClimberSubsystem(){ //TODO: Get CAN IDs
        leftClimber = new TalonFX(0);
        rightClimber = new TalonFX(0);
        applyConfigs();
    }

    private void applyConfigs(){
        LEFT_TALONFX_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        RIGHT_TALONFX_CONFIG = LEFT_TALONFX_CONFIG;

        leftClimber.getConfigurator().apply(LEFT_TALONFX_CONFIG);
        rightClimber.getConfigurator().apply(RIGHT_TALONFX_CONFIG);

    }
}
