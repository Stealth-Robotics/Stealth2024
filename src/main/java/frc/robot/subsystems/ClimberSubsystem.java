package frc.robot.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberSubsystem {
    private final TalonFX leftClimber;
    private final TalonFX rightClimber;
    
    public ClimberSubsystem(){ //TODO: Get CAN IDs
        leftClimber = new TalonFX(0);
        rightClimber = new TalonFX(0);
    }
}
