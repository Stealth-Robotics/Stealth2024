package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class IntakeSubsystem extends SubsystemBase {

    private final TalonFX intakeMotor;

    private final DigitalInput rightBeamBreak;
    private final DigitalInput leftBeamBreak;

    public IntakeSubsystem() {
        intakeMotor = new TalonFX(0); 
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
        //intakeMotor.setInverted(true);
        rightBeamBreak = new DigitalInput(0);
        leftBeamBreak = new DigitalInput(0);
    }

    public void setMotorSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public boolean getRightBeamBreak() {
        return rightBeamBreak.get();
    }

    public boolean getLeftBeamBreak() {
        return leftBeamBreak.get();
    }
}
