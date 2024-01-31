package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RotatorSubsystem extends SubsystemBase {
    // TODO: find gear ratio once CAD does it
    private final double ROTATOR_GEAR_RATIO = 1;
    private final double ROTATOR_POSITION_COEFFICIENT = (2 * Math.PI) / (2048 * ROTATOR_GEAR_RATIO);

    private final TalonFX rotatorMotorOne;
    private final TalonFX rotatorMotorTwo;
    private final TalonFX rotatorMotorThree;

    private double kS = 0.0;
    private double kV = 0.0;
    private double kA = 0.0;

    private double kP = 0.0;
    private double kI = 0.0;
    private double kD = 0.0;

    // tolerance in radians
    // TODO: TUNE THIS
    private final double kTolerance = 0.0;

    private double MOTION_MAGIC_ACCELERATION = 0.0;

    private final TalonFXConfiguration ROTATOR_CONFIG_ONE = new TalonFXConfiguration();

    private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    public RotatorSubsystem() {
        rotatorMotorOne = new TalonFX(0);
        rotatorMotorTwo = new TalonFX(0);
        rotatorMotorThree = new TalonFX(0);

        applyConfigs();

        throw new UnsupportedOperationException();
    }

    private void applyConfigs() {
        ROTATOR_CONFIG_ONE.Slot0.kS = kS;
        ROTATOR_CONFIG_ONE.Slot0.kV = kV;
        ROTATOR_CONFIG_ONE.Slot0.kA = kA;

        ROTATOR_CONFIG_ONE.Slot0.kP = kP;
        ROTATOR_CONFIG_ONE.Slot0.kI = kI;
        ROTATOR_CONFIG_ONE.Slot0.kD = kD;

        ROTATOR_CONFIG_ONE.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;

        TalonFXConfiguration ROTATOR_CONFIG_TWO = ROTATOR_CONFIG_ONE;
        TalonFXConfiguration ROTATOR_CONFIG_THREE = ROTATOR_CONFIG_ONE;

        // TODO: CHECK THESE DIRECTIONS
        ROTATOR_CONFIG_ONE.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        ROTATOR_CONFIG_TWO.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        ROTATOR_CONFIG_THREE.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        rotatorMotorOne.getConfigurator().apply(ROTATOR_CONFIG_ONE);
        rotatorMotorTwo.getConfigurator().apply(ROTATOR_CONFIG_TWO);
        rotatorMotorThree.getConfigurator().apply(ROTATOR_CONFIG_THREE);

        rotatorMotorTwo.setControl(new Follower(rotatorMotorOne.getDeviceID(), false));
        rotatorMotorThree.setControl(new Follower(rotatorMotorOne.getDeviceID(), false));

    }

    private void setMotorTargetPosition(double angRad) {
        motionMagicVoltage.Position = angRad / ROTATOR_POSITION_COEFFICIENT;
        rotatorMotorOne.setControl(motionMagicVoltage);
    }

    private double getMotorPosition() {
        return rotatorMotorOne.getPosition().getValueAsDouble() * ROTATOR_POSITION_COEFFICIENT;
    }

    private boolean isMotorAtTarget() {
        return Math.abs(getMotorPosition() - motionMagicVoltage.Position) <= kTolerance;
    }

    public Command RotateToPositionCommand(double angRad) {
        return this.runOnce(() -> setMotorTargetPosition(angRad)).until(() -> this.isMotorAtTarget());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.setSmartDashboardType("rotator gains");

        builder.addDoubleProperty("ks", () -> this.kS, (value) -> {
            this.kS = value;
            applyConfigs();
        });

        builder.addDoubleProperty("kv", () -> this.kV, (value) -> {
            this.kV = value;
            applyConfigs();
        });

        builder.addDoubleProperty("ka", () -> this.kA, (value) -> {
            this.kA = value;
            applyConfigs();
        });

        builder.addDoubleProperty("kp", () -> this.kP, (value) -> {
            this.kP = value;
            applyConfigs();
        });

        builder.addDoubleProperty("ki", () -> this.kI, (value) -> {
            this.kI = value;
            applyConfigs();
        });

        builder.addDoubleProperty("kd", () -> this.kD, (value) -> {
            this.kD = value;
            applyConfigs();
        });
    }

}
