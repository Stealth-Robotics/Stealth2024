package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RotatorSubsystem extends SubsystemBase {

    private enum NeutralMode {
        COAST,
        BRAKE;
    }

    // TODO: find gear ratio once CAD does it

    // Explantation: Gear ration = how many turns of the motor shaft = 1 full
    // revolution of the arm
    // in applyConfigs, we specify sensor to mechanism ratio as gear ratio. This
    // makes it so the sensor returns a value between 0 and 1
    // with 0 being with the arm straight out, and 1 being 1 full revolution of the
    // arm
    // this means that with ROTATOR_POSITION_COEFFICIENT, we convert this ouput from
    // being 0 to 1, to instead being 0 to 2 pi radians
    private final double ROTATOR_GEAR_RATIO = 1;
    private final double ROTATOR_POSITION_COEFFICIENT = 2 * Math.PI;

    private final TalonFX rotatorMotorOne;
    private final TalonFX rotatorMotorTwo;
    private final TalonFX rotatorMotorThree;

    private double kS = 0.0;
    private double kV = 0.0;
    private double kG = 0.0;

    private double kP = 0.0;
    private double kI = 0.0;
    private double kD = 0.0;

    private NeutralMode motorMode = NeutralMode.COAST;

    // tolerance in radians
    // TODO: TUNE THIS
    private final double kTolerance = 0.0;

    private double MOTION_MAGIC_ACCELERATION = 0.0;
    private double MOTION_MAGIC_VELOCITY = 0.0;

    private final TalonFXConfiguration ROTATOR_CONFIG_ONE = new TalonFXConfiguration();

    private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    public RotatorSubsystem() {
        rotatorMotorOne = new TalonFX(0);
        rotatorMotorTwo = new TalonFX(0);
        rotatorMotorThree = new TalonFX(0);

        setMotorsToCoast();

        applyConfigs();

        throw new UnsupportedOperationException();
    }

    private void applyConfigs() {
        ROTATOR_CONFIG_ONE.Slot0.kS = kS;
        ROTATOR_CONFIG_ONE.Slot0.kV = kV;
        ROTATOR_CONFIG_ONE.Slot0.kG = kG;

        ROTATOR_CONFIG_ONE.Slot0.kP = kP;
        ROTATOR_CONFIG_ONE.Slot0.kI = kI;
        ROTATOR_CONFIG_ONE.Slot0.kD = kD;

        ROTATOR_CONFIG_ONE.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
        ROTATOR_CONFIG_ONE.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_VELOCITY;

        ROTATOR_CONFIG_ONE.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        ROTATOR_CONFIG_ONE.Feedback.SensorToMechanismRatio = ROTATOR_GEAR_RATIO;

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

    // these methods will only be used with the buttons
    public void setMotorsToCoast() {
        rotatorMotorOne.setNeutralMode(NeutralModeValue.Coast);
        rotatorMotorTwo.setNeutralMode(NeutralModeValue.Coast);
        rotatorMotorThree.setNeutralMode(NeutralModeValue.Coast);
    }

    public void setMotorsToBrake() {
        rotatorMotorOne.setNeutralMode(NeutralModeValue.Brake);
        rotatorMotorTwo.setNeutralMode(NeutralModeValue.Brake);
        rotatorMotorThree.setNeutralMode(NeutralModeValue.Brake);
    }

    public Command toggleMotorMode() {
        return this.runOnce(() -> {
            if (motorMode == NeutralMode.COAST) {
                setMotorsToBrake();
                motorMode = NeutralMode.BRAKE;
            }
            if (motorMode == NeutralMode.BRAKE) {
                setMotorsToCoast();
                motorMode = NeutralMode.COAST;
            }
        }).ignoringDisable(true);
    }

    private void setMotorTargetPosition(double angRad) {
        motionMagicVoltage.Position = angRad / ROTATOR_POSITION_COEFFICIENT;
        rotatorMotorOne.setControl(motionMagicVoltage);
    }

    private double getMotorPosition() {
        return rotatorMotorOne.getRotorPosition().getValueAsDouble() * ROTATOR_POSITION_COEFFICIENT;
    }

    public void resetEncoder() {
        rotatorMotorOne.setPosition(0);
    }

    private double getTargetPosition() {
        return motionMagicVoltage.Position * ROTATOR_POSITION_COEFFICIENT;
    }

    private boolean isMotorAtTarget() {
        return Math.abs(getMotorPosition() - getTargetPosition()) <= kTolerance;
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

        builder.addDoubleProperty("kg", () -> this.kG, (value) -> {
            this.kG = value;
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
