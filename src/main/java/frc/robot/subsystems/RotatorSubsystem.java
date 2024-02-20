package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RotatorSubsystem extends SubsystemBase {

    // Explantation: Gear ration = how many turns of the motor shaft = 1 full
    // revolution of the arm
    // in applyConfigs, we specify sensor to mechanism ratio as gear ratio. This
    // makes it so the sensor returns a value between 0 and 1
    // with 0 being with the arm straight out, and 1 being 1 full revolution of the
    // arm
    // this means that with ROTATOR_POSITION_COEFFICIENT, we convert this ouput from
    // being 0 to 1, to instead being 0 to 2 pi radians
    private final double ROTATOR_GEAR_RATIO = (80.0 / 10.0) * (82.0 / 18.0) * (52.0 / 15.0);
    private final double ROTATOR_POSITION_COEFFICIENT = 2 * Math.PI;
    // the offset of the home position and straight out on the arm. I.E. what should
    // the encoder read when the arm is on the hard stop?
    private final double ZERO_OFFSET = Math.toRadians(-0.79);

    private final TalonFX rotatorMotorOne;
    private final TalonFX rotatorMotorTwo;

    private double kS = 0.0;
    private double kV = 0.0;
    private double kG = 0;

    private double kP = 60;
    private double kI = 0.0;
    private double kD = 0.0;

    private DigitalInput homeButton = new DigitalInput(0);
    private DigitalInput toggleMotorModeButton = new DigitalInput(1);

    private NeutralModeValue motorMode = NeutralModeValue.Coast;
    private boolean isHomed = false;

    // tolerance in radians
    // TODO: TUNE THIS
    // this is a tolerance of 1 degree
    private final double kTOLERANCE = Math.toRadians(1);

    private final double MOTION_MAGIC_JERK = 0.0;
    private double MOTION_MAGIC_ACCELERATION = 50;
    private double MOTION_MAGIC_CRUISE_VELOCITY = 30;

    private final TalonFXConfiguration ROTATOR_MOTOR_CONFIG = new TalonFXConfiguration();

    private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);
    private DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

    Path tempPath = Paths.get("/tmp/rotatorHomed.txt");

    public RotatorSubsystem() {
        rotatorMotorOne = new TalonFX(14);
        rotatorMotorTwo = new TalonFX(15);

        isHomed = Files.exists(tempPath);
        motorMode = isHomed ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        applyConfigs();
        // throw new UnsupportedOperationException("Test rotator code");

        
    }

    private void applyConfigs() {
        ROTATOR_MOTOR_CONFIG.Slot0.kS = kS;
        ROTATOR_MOTOR_CONFIG.Slot0.kV = kV;

        ROTATOR_MOTOR_CONFIG.Slot0.kG = kG;

        ROTATOR_MOTOR_CONFIG.Slot0.kP = kP;
        ROTATOR_MOTOR_CONFIG.Slot0.kI = kI;
        ROTATOR_MOTOR_CONFIG.Slot0.kD = kD;

        ROTATOR_MOTOR_CONFIG.MotionMagic.MotionMagicJerk = MOTION_MAGIC_JERK;
        ROTATOR_MOTOR_CONFIG.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
        ROTATOR_MOTOR_CONFIG.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_CRUISE_VELOCITY;

        ROTATOR_MOTOR_CONFIG.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        ROTATOR_MOTOR_CONFIG.Feedback.SensorToMechanismRatio = ROTATOR_GEAR_RATIO;

        // TODO: CHECK THIS DIRECTIONS
        ROTATOR_MOTOR_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        

        ROTATOR_MOTOR_CONFIG.MotorOutput.NeutralMode = motorMode;

        rotatorMotorOne.getConfigurator().apply(ROTATOR_MOTOR_CONFIG);
        rotatorMotorTwo.getConfigurator().apply(ROTATOR_MOTOR_CONFIG);

        rotatorMotorTwo.setControl(new Follower(rotatorMotorOne.getDeviceID(), false));

    }

    // these methods will only be used with the buttons
    private void setMotorsToCoast() {
        rotatorMotorOne.setNeutralMode(NeutralModeValue.Coast);
        rotatorMotorTwo.setNeutralMode(NeutralModeValue.Coast);

    }

    private void setMotorsToBrake() {
        rotatorMotorOne.setNeutralMode(NeutralModeValue.Brake);
        rotatorMotorTwo.setNeutralMode(NeutralModeValue.Brake);
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

        if (newValue) {
            if (!Files.exists(tempPath)) {
                try {
                    Files.createFile(tempPath);
                } catch (IOException e) {
                    e.printStackTrace();
                }
            } else {
                // Already homed, variable is not set again, but encoder is reset again by
                // homeArmCommand
            }

        } else {
            if (Files.exists(tempPath)) {
                try {
                    Files.delete(tempPath);
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }

            else {
                // Already not homed, variable is not set again
            }
        }
    }

    public Command homeArmCommand() {
        return new InstantCommand(() -> {
            resetEncoder();
            setHomed(true);
        }, this).ignoringDisable(true);
    }

    public Command armManualControl(DoubleSupplier manualControlSupplier) {
        return this.runOnce(() -> setDutyCycle(manualControlSupplier.getAsDouble()));
    }

    private void setDutyCycle(double dutyCycle) {
        dutyCycleOut.Output = dutyCycle;
        rotatorMotorOne.setControl(dutyCycleOut);
    }

    public void holdCurrentPosition() {
        setMotorTargetPosition(getMotorPosition());
    }

    public void resetEncoder() {
        rotatorMotorOne.setPosition(ZERO_OFFSET / ROTATOR_POSITION_COEFFICIENT);
        setMotorTargetPosition(ZERO_OFFSET);
        // applyConfigs();
    }

    private double getMotorPosition() {
        return rotatorMotorOne.getPosition().getValueAsDouble() * ROTATOR_POSITION_COEFFICIENT;
    }

    private double getTargetPosition() {
        return motionMagicVoltage.Position;
    }

    private boolean isMotorAtTarget() {
        return Math.abs(getMotorPosition() - getTargetPosition()) <= kTOLERANCE;
    }

    private void setMotorTargetPosition(double angRad) {
        motionMagicVoltage.Position = angRad / ROTATOR_POSITION_COEFFICIENT;
        rotatorMotorOne.setControl(motionMagicVoltage);
    }

    public Command rotateToPositionCommand(double angRad) {
        return this.runOnce(() -> setMotorTargetPosition(angRad)).until(() -> this.isMotorAtTarget());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        

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
    @Override
    public void periodic() {
        System.out.println("sp: " + Math.toDegrees(getTargetPosition()));
        System.out.println("pos: " + Math.toDegrees(getMotorPosition()));
    }

}
