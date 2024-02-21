package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestRotator extends SubsystemBase{

    private final TalonFX motor1;
    private final TalonFX motor2;

    private final double GEAR_RATIO = (80.0 / 10.0) * (82.0 / 18.0) * (52.0 / 15.0);

    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    private NeutralModeValue motorMode = NeutralModeValue.Coast;

    private DigitalInput homeButton = new DigitalInput(0);
    private DigitalInput toggleMotorModeButton = new DigitalInput(1);

    private boolean isHomed = false;



    public TestRotator(){
        motor1 = new TalonFX(14);
        motor2 = new TalonFX(15);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotionMagic.MotionMagicAcceleration = 50;
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = 30;
        motorConfig.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        motorConfig.Slot0.kP = 60;

        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motor1.getConfigurator().apply(motorConfig);
        motor2.getConfigurator().apply(motorConfig);

        motor2.setControl(new Follower(motor1.getDeviceID(), false));
    }

    private double getPosition(){
        return motor1.getPosition().getValueAsDouble();
    }

    private void resetEncoder(){
        motor1.setPosition(0);
    }

    // these methods will only be used with the buttons
    private void setMotorsToCoast() {
        motor1.setNeutralMode(NeutralModeValue.Coast);
        motor2.setNeutralMode(NeutralModeValue.Coast);

    }

    private void setMotorsToBrake() {
        motor1.setNeutralMode(NeutralModeValue.Brake);
        motor2.setNeutralMode(NeutralModeValue.Brake);
    }

    public boolean getToggleMotorModeButton() {
        return !toggleMotorModeButton.get();
    }

    public NeutralModeValue getMotorMode() {
        return motorMode;
    }

    public Command toggleMotorModeCommand() {
        return new InstantCommand(
                () -> {
                    if (motorMode == NeutralModeValue.Coast) {
                        setMotorsToBrake();
                        motorMode = NeutralModeValue.Brake;
                    } else {
                        setMotorsToCoast();
                        motorMode = NeutralModeValue.Coast;
                    }
                }, this).ignoringDisable(true);
    }

    public boolean getHomeButton() {
        return !homeButton.get();
    }

    public boolean isHomed() {
        return isHomed;
    }

    public void setHomed(boolean newValue) {
        this.isHomed = newValue;
    }

    public Command homeArmCommand() {
        return new InstantCommand(() -> {
            resetEncoder();
            setHomed(true);
        }, this).ignoringDisable(true);
    }

    @Override
    public void periodic() {
        System.out.println("pos: " + getPosition());
    }

    
}
