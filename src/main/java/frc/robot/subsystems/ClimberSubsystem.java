package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    private final TalonFX leftClimber;
    private final TalonFX rightClimber;

    TalonFXConfiguration CLIMBER_CONFIG = new TalonFXConfiguration();
    
    public ClimberSubsystem(){ //TODO: Get CAN IDs
        leftClimber = new TalonFX(18);
        rightClimber = new TalonFX(19);

        applyConfigs();

        leftClimber.optimizeBusUtilization();
        rightClimber.optimizeBusUtilization();
    }

    private void applyConfigs(){
        CLIMBER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        CLIMBER_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        leftClimber.getConfigurator().apply(CLIMBER_CONFIG);
        rightClimber.getConfigurator().apply(CLIMBER_CONFIG);

        rightClimber.setControl(new Follower(leftClimber.getDeviceID(), false));
    }

    public void setPower(double power){
        leftClimber.set(power);
        rightClimber.set(power);
    }
}
