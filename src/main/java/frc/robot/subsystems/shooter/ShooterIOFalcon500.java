package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterIOFalcon500 implements ShooterIO {

    // Hardware
    private final TalonFX leftShooter;
    private final TalonFX rightShooter;

    // Status Signals
    private final StatusSignal<Double> leftShooterVelocity;
    private final StatusSignal<Double> leftShooterAppliedVolts;
    private final StatusSignal<Double> leftShooterOutputCurrent;
    private final StatusSignal<Double> leftShooterTemp;

    private final StatusSignal<Double> rightShooterVelocity;
    private final StatusSignal<Double> rightShooterAppliedVolts;
    private final StatusSignal<Double> rightShooterOutputCurrent;
    private final StatusSignal<Double> rightShooterTemp;

    // Config
    TalonFXConfiguration motorConfig;
    Slot0Configs controllerConfig;

    // Control Requests
    private VoltageOut voltageOut = new VoltageOut(0.0);
    private NeutralOut neutralOut = new NeutralOut();
    private MotionMagicVelocityVoltage motionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0.0);

    public ShooterIOFalcon500() {
        this.leftShooter = new TalonFX(config.leftMotorID());
        this.rightShooter = new TalonFX(config.rightMotorID());

        motorConfig.MotorOutput.NeutralMode = config.neutralMode();
        motorConfig.MotorOutput.Inverted = config.inverted();

        motorConfig.MotionMagic.MotionMagicAcceleration = gains.MOTION_MAGIC_ACCELERATION();
        motorConfig.MotionMagic.MotionMagicJerk = gains.MOTION_MAGIC_JERK();

        controllerConfig.kS = gains.kS();
        controllerConfig.kV = gains.kV();
        controllerConfig.kA = gains.kA();

        controllerConfig.kP = gains.kP();
        controllerConfig.kI = gains.kI();
        controllerConfig.kD = gains.kD();

        motorConfig.Slot0 = controllerConfig;

        leftShooter.getConfigurator().apply(motorConfig);
        rightShooter.getConfigurator().apply(motorConfig);

        leftShooterVelocity = leftShooter.getVelocity();
        leftShooterAppliedVolts = leftShooter.getMotorVoltage();
        leftShooterOutputCurrent = leftShooter.getTorqueCurrent();
        leftShooterTemp = leftShooter.getDeviceTemp();

        rightShooterVelocity = rightShooter.getVelocity();
        rightShooterAppliedVolts = rightShooter.getMotorVoltage();
        rightShooterOutputCurrent = rightShooter.getTorqueCurrent();
        rightShooterTemp = rightShooter.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                100.0,
                leftShooterVelocity,
                leftShooterAppliedVolts,
                leftShooterOutputCurrent,
                leftShooterTemp,
                rightShooterVelocity,
                rightShooterAppliedVolts,
                rightShooterOutputCurrent,
                rightShooterTemp);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.leftShooterConnected = BaseStatusSignal
                .refreshAll(
                        leftShooterVelocity,
                        leftShooterAppliedVolts,
                        leftShooterOutputCurrent,
                        leftShooterTemp)
                .isOK();

        inputs.rightShooterConnected = BaseStatusSignal
                .refreshAll(
                        rightShooterVelocity,
                        rightShooterAppliedVolts,
                        rightShooterOutputCurrent,
                        rightShooterTemp)
                .isOK();

        inputs.leftShooterVelocity = leftShooterVelocity.getValue() * 60.0;
        inputs.leftShooterAppliedVolts = leftShooterAppliedVolts.getValue();
        inputs.leftShooterOutputCurrent = leftShooterOutputCurrent.getValue();
        inputs.leftShooterTempCelsius = leftShooterTemp.getValue();

        inputs.rightShooterVelocity = rightShooterVelocity.getValue() * 60.0;
        inputs.rightShooterAppliedVolts = rightShooterAppliedVolts.getValue();
        inputs.rightShooterOutputCurrent = rightShooterOutputCurrent.getValue();
        inputs.rightShooterTempCelsius = rightShooterTemp.getValue();
    }

    @Override
    public void setLeftShooterVolts(double volts) {
        leftShooter.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void setRightShooterVolts(double volts) {
        rightShooter.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void stopShooter() {
        leftShooter.setControl(neutralOut);
        rightShooter.setControl(neutralOut);
    }

    @Override
    public void setLeftShooterVelocity(double velocityRPM) {
        leftShooter.setControl(motionMagicVelocityVoltage.withVelocity(velocityRPM / 60.0));
    }

    @Override
    public void setRightShooterVelocity(double velocityRPM) {
        rightShooter.setControl(motionMagicVelocityVoltage.withVelocity(velocityRPM / 60.0));
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        controllerConfig.kP = kP;
        controllerConfig.kI = kI;
        controllerConfig.kD = kD;

        leftShooter.getConfigurator().apply(controllerConfig);
        rightShooter.getConfigurator().apply(controllerConfig);
    }

    @Override
    public void setFF(double kS, double kV, double kA) {
        controllerConfig.kS = kS;
        controllerConfig.kV = kV;
        controllerConfig.kA = kA;

        leftShooter.getConfigurator().apply(controllerConfig);
        rightShooter.getConfigurator().apply(controllerConfig);
    }
}
