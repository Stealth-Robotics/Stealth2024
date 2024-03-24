package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.opencv.video.DenseOpticalFlow;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class IntakeSubsystem extends SubsystemBase {

    private final TalonFX intakeMotor;
    private final TalonFX deployMotor;

    private final DigitalInput frontBeamBreak;
    private final DigitalInput backBeamBreak;

    private PositionVoltage deployPositionVoltage = new PositionVoltage(0).withSlot(0);

    // TODO: tune these values
    private final double kP = 0.0;
    private final double kI = 0.0;
    private final double kD = 0.0;

    //second set of gains to allow intake to retract a bit when hit. these (especially kP) should be lower than the other gains
    private final double deployed_kP = 0.0;
    private final double deployed_kI = 0.0;
    private final double deployed_kD = 0.0;

    private final double kTolerance = 0.0;

    public IntakeSubsystem() {
        // TODO: GET CAN IDs
        intakeMotor = new TalonFX(20);
        deployMotor = new TalonFX(-1);
        frontBeamBreak = new DigitalInput(2);
        backBeamBreak = new DigitalInput(3);

        intakeMotor.getPosition().setUpdateFrequency(4);

        applyConfigs();
    }

    private void applyConfigs() {
        TalonFXConfiguration INTAKE_MOTOR_CONFIG = new TalonFXConfiguration();
        TalonFXConfiguration DEPLOY_MOTOR_CONFIG = new TalonFXConfiguration();
        INTAKE_MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        INTAKE_MOTOR_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // TODO: Change if needed
        
        
        DEPLOY_MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        DEPLOY_MOTOR_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // TODO: Change if needed

        DEPLOY_MOTOR_CONFIG.Slot0.kP = kP;
        DEPLOY_MOTOR_CONFIG.Slot0.kI = kI;
        DEPLOY_MOTOR_CONFIG.Slot0.kD = kD;

        DEPLOY_MOTOR_CONFIG.Slot1.kP = deployed_kP;
        DEPLOY_MOTOR_CONFIG.Slot1.kI = deployed_kI;
        DEPLOY_MOTOR_CONFIG.Slot1.kD = deployed_kD;

        intakeMotor.getConfigurator().apply(INTAKE_MOTOR_CONFIG);
        deployMotor.getConfigurator().apply(DEPLOY_MOTOR_CONFIG);
    
    }
    //Position in rotations
    private void setDeployTarget(double position) {
        deployPositionVoltage.Position = position;
        deployMotor.setControl(deployPositionVoltage);
        
    }

    private boolean getDeployAtSetpoint() {
        return Math.abs(deployMotor.getPosition().getValueAsDouble() - deployPositionVoltage.Position) <= kTolerance;
    }

    public Command deployIntakeCommand() {
        //TODO: tune position
        return this.runOnce(() -> setDeployTarget(0))
                .andThen(new WaitUntilCommand(() -> getDeployAtSetpoint()))
                .andThen(new InstantCommand(() -> setDeployedGains(), this));
    }

    public Command retractIntakeCommand() {
        //TODO: tune position
        return this.runOnce(() -> setDeployTarget(0))
                .andThen(new WaitUntilCommand(() -> getDeployAtSetpoint()));
    }

    private void setDeployedGains() {
        final PositionVoltage m_request = new PositionVoltage(deployMotor.getPosition().getValueAsDouble()).withSlot(1);
        deployMotor.setControl(m_request);
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public Command homeDeployCommand() {
        return new ConditionalCommand(
                new InstantCommand(() -> {

                    deployMotor.setPosition(0);
                    setDeployTarget(0);

                }, this).ignoringDisable(true),
                new PrintCommand("driver station is enabled").ignoringDisable(true),
                () -> DriverStation.isDisabled());

    }

    public Command intakeCommand(DoubleSupplier speed) {
        return this.run(()->setIntakeSpeed(speed.getAsDouble()))
        .until(() -> Math.abs(speed.getAsDouble()) < 0.1)
        .finallyDo(() -> setIntakeSpeed(0));
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
