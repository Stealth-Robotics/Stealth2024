package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem {
    private final TalonFX leftClimber;
    private final TalonFX rightClimber;

    TalonFXConfiguration CLIMBER_CONFIG = new TalonFXConfiguration();
    
    public ClimberSubsystem(){ //TODO: Get CAN IDs
        leftClimber = new TalonFX(18);
        rightClimber = new TalonFX(19);
        applyConfigs();
    }

    private void applyConfigs(){
        CLIMBER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        CLIMBER_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        leftClimber.getConfigurator().apply(CLIMBER_CONFIG);
        rightClimber.getConfigurator().apply(CLIMBER_CONFIG);

    }

    private void setPower(double power){
        leftClimber.set(power);
        rightClimber.set(power);
    }

    public Command setPowerCommand(double power){
       return new InstantCommand(()-> setPower(power));

    }
}
