package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsystem extends SubsystemBase {

    private final TalonFX intakeMotor;

    private final TimeOfFlight frontDistanceSensor;
    private final TimeOfFlight backDistanceSensor;

    public IntakeSubsystem() {
        //TODO: GET CAN IDs
        intakeMotor = new TalonFX(20); 
        frontDistanceSensor = new TimeOfFlight(-1);
        backDistanceSensor = new TimeOfFlight(-1);
        frontDistanceSensor.setRangingMode(RangingMode.Short, 20);
        backDistanceSensor.setRangingMode(RangingMode.Short, 20);
        applyConfigs();
    }

    private void applyConfigs() {
        TalonFXConfiguration INTAKE_MOTOR_CONFIG = new TalonFXConfiguration();
        INTAKE_MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        INTAKE_MOTOR_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //TODO: Change if needed
        intakeMotor.getConfigurator().apply(INTAKE_MOTOR_CONFIG);
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }


    //TODO: tune these values. they are in mm.
    public boolean isRingAtFrontOfIntake() {
        return frontDistanceSensor.getRange() < 30;
    }

    public boolean isRingFullyInsideIntake() {
        return backDistanceSensor.getRange() < 30;
    }

    
}