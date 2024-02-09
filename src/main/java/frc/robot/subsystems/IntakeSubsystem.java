package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class IntakeSubsystem extends SubsystemBase {

    private final TalonFX intakeMotor;

    private final DigitalInput frontBeamBreak;
    private final DigitalInput backBeamBreak;

    public IntakeSubsystem() {
        intakeMotor = new TalonFX(0); 
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
        //intakeMotor.setInverted(true);
        frontBeamBreak = new DigitalInput(0);
        backBeamBreak = new DigitalInput(0);
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public void setOuttakeSpeed(double speed) {
        intakeMotor.set(-speed);
    }

    public void stop() {
        intakeMotor.set(0);
    }

    public boolean isRingAtFrontOfIntake() {
        return frontBeamBreak.get();
    }

    public boolean isRingFullyInsideIntake() {
        return backBeamBreak.get();
    }
}
