package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class RotatorSubsystem extends SubsystemBase {

    private enum RotatorState {
        OUTSIDE_UPPER_LIMIT,
        INSIDE_LIMIT
    }

    // Explantation: Gear ration = how many turns of the motor shaft = 1 full
    // revolution of the arm
    // in applyConfigs, we specify sensor to mechanism ratio as gear ratio. This
    // makes it so the sensor returns a value between 0 and 1
    // with 0 being with the arm straight out, and 1 being 1 full revolution of the
    // arm
    private final double ROTATOR_GEAR_RATIO = (80.0 / 10.0) * (82.0 / 18.0) * (52.0 / 15.0);
    // the offset of the home position and straight out on the arm. I.E. what should
    // the encoder read when the arm is on the hard stop?
    private final double ZERO_OFFSET = -0.0009765;

    private final TalonFX rotatorMotorOne;
    private final TalonFX rotatorMotorTwo;

    private double kS = 0.0;
    private double kV = 14.8;
    private double kG = 0.3;

    private double kP = 30;
    private double kI = 10;
    private double kD = 1;

    private DigitalInput homeButton = new DigitalInput(0);
    private DigitalInput toggleMotorModeButton = new DigitalInput(1);

    private NeutralModeValue motorMode = NeutralModeValue.Coast;
    private boolean isHomed = false;

    // tolerance in radians
    // TODO: TUNE THIS
    // this is a tolerance of 1 degree
    private final double kTOLERANCE = Units.degreesToRotations(0.75);

    private final double MOTION_MAGIC_JERK = 6;
    private double MOTION_MAGIC_ACCELERATION = 2;
    private double MOTION_MAGIC_CRUISE_VELOCITY = 0.5;

    private final TalonFXConfiguration ROTATOR_MOTOR_CONFIG = new TalonFXConfiguration();

    private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);
    private DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

    private RotatorState rotatorState = RotatorState.INSIDE_LIMIT;

    private GenericEntry ampOutIntake;

    public RotatorSubsystem() {

        ampOutIntake = Shuffleboard.getTab("SmartDashboard").add("amp out intake?", false)
                .withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
        rotatorMotorOne = new TalonFX(14);
        rotatorMotorTwo = new TalonFX(15);

        applyConfigs();
        // throw new UnsupportedOperationException("Test rotator code");

        resetEncoder();
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
        ROTATOR_MOTOR_CONFIG.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        ROTATOR_MOTOR_CONFIG.ClosedLoopGeneral.ContinuousWrap = false;

        ROTATOR_MOTOR_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        ROTATOR_MOTOR_CONFIG.MotorOutput.NeutralMode = motorMode;

        rotatorMotorOne.getConfigurator().apply(ROTATOR_MOTOR_CONFIG);
        rotatorMotorTwo.getConfigurator().apply(ROTATOR_MOTOR_CONFIG);

        rotatorMotorTwo.setControl(new Follower(rotatorMotorOne.getDeviceID(), false));
    }

    // these methods will only be used with the buttons
    public void setMotorsToCoast() {
        motorMode = NeutralModeValue.Coast;
        applyConfigs();

    }

    private void setMotorsToBrake() {
        motorMode = NeutralModeValue.Brake;
        applyConfigs();
    }

    public boolean getToggleMotorModeButton() {
        return !toggleMotorModeButton.get();
    }

    public NeutralModeValue getMotorMode() {
        return motorMode;
    }

    public Command toggleMotorModeCommand() {
        return new ConditionalCommand(new InstantCommand(
                () -> {
                    if (motorMode == NeutralModeValue.Coast) {
                        setMotorsToBrake();
                        motorMode = NeutralModeValue.Brake;
                    } else {
                        setMotorsToCoast();
                        motorMode = NeutralModeValue.Coast;
                    }

                }, this).ignoringDisable(true),
                new PrintCommand("driver station is enabled").ignoringDisable(true),
                () -> DriverStation.isDisabled());

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
        return new ConditionalCommand(
                new InstantCommand(() -> {

                    resetEncoder();
                    setHomed(true);

                }, this).ignoringDisable(true),
                new PrintCommand("driver station is enabled").ignoringDisable(true),
                () -> DriverStation.isDisabled());

    }

    public Command armManualControl(DoubleSupplier manualControlSupplier, Supplier<RotatorState> rotatorStateSupplier) {
        return this.runOnce(() -> {
            double manualControl = manualControlSupplier.getAsDouble();
            if (rotatorStateSupplier.get() == RotatorState.OUTSIDE_UPPER_LIMIT) {
                manualControl = MathUtil.clamp(manualControl, -0.5, 0);
            } else {
                manualControl = MathUtil.clamp(manualControl, -0.5, 0.5);
            }
            setDutyCycle(manualControl);
        });
    }

    public void setDutyCycle(double dutyCycle) {
        dutyCycleOut.Output = dutyCycle;
        rotatorMotorOne.setControl(dutyCycleOut);

    }

    public void holdCurrentPosition() {
        setMotorTargetPosition(getMotorPosition());
    }

    public void resetEncoder() {
        rotatorMotorOne.setPosition(ZERO_OFFSET);
        setMotorTargetPosition(ZERO_OFFSET);
    }

    private double getMotorPosition() {
        return rotatorMotorOne.getPosition().getValueAsDouble();
    }

    private double getTargetPosition() {
        return motionMagicVoltage.Position;
    }

    public boolean isMotorAtTarget() {
        return Math.abs(getMotorPosition() - getTargetPosition()) <= kTOLERANCE;
    }

    public void setMotorTargetPosition(double rotations) {
        motionMagicVoltage.Position = rotations;
        rotatorMotorOne.setControl(motionMagicVoltage);
    }

    public Command rotateToPositionCommand(double rotations) {
        return new InstantCommand(() -> setMotorTargetPosition(rotations), this)
                .andThen(new WaitUntilCommand(this::isMotorAtTarget));
    }

    public Supplier<RotatorState> getRotatorState() {
        return () -> rotatorState;
    }

    public BooleanSupplier getAmpOutIntake() {
        return () -> ampOutIntake.getBoolean(false);
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("rotator", Units.rotationsToDegrees(getMotorPosition()));
        SmartDashboard.putNumber("rotator target", Units.rotationsToDegrees(getTargetPosition()));

        if (Units.rotationsToDegrees(getMotorPosition()) >= 90) {
            rotatorState = RotatorState.OUTSIDE_UPPER_LIMIT;
        }

        else {
            rotatorState = RotatorState.INSIDE_LIMIT;
        }
        // rotatorMotorTwo.setControl(new
        // VoltageOut(rotatorMotorOne.getMotorVoltage().getValueAsDouble()));
    }
}
