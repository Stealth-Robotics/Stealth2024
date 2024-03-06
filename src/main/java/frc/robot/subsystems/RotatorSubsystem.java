package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

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
    private double kV = 15.5;
    private double kG = 0.3;

    private double kP = 120;
    private double kI = 90;
    private double kD = 20;

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

    StatusSignal<Double> rotatorPosition;

    StatusSignal<Double> motor1SupplyCurrent;
    StatusSignal<Double> motor2SupplyCurrent;

    private RotatorState rotatorState = RotatorState.INSIDE_LIMIT;

    public RotatorSubsystem() {
        rotatorMotorOne = new TalonFX(15);
        rotatorMotorTwo = new TalonFX(14);

        motorMode = isHomed ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        applyConfigs();
        // throw new UnsupportedOperationException("Test rotator code");

        rotatorPosition = rotatorMotorOne.getPosition();
        rotatorPosition.setUpdateFrequency(500);

        motor1SupplyCurrent = rotatorMotorOne.getSupplyCurrent();
        motor1SupplyCurrent.setUpdateFrequency(100);

        motor2SupplyCurrent = rotatorMotorTwo.getSupplyCurrent();
        motor2SupplyCurrent.setUpdateFrequency(100);

        rotatorMotorOne.optimizeBusUtilization();
        rotatorMotorTwo.optimizeBusUtilization();

        resetEncoder();

        if (Constants.TUNING_MODE) {
            ShuffleboardTab rotatorTab = Shuffleboard.getTab("Rotator");
            rotatorTab.add("Motor 1", rotatorMotorOne);
            rotatorTab.add("Motor 2", rotatorMotorTwo);

            rotatorTab.add("Home Button", homeButton);
            rotatorTab.addBoolean("Rotator Homed", this::isHomed);
            rotatorTab.add("Toggle Motor Mode Button", toggleMotorModeButton);
            rotatorTab.addString("Motor Mode", () -> getMotorMode().toString());

            rotatorTab.addNumber("Rotator Position - Rotations (Raw)", this::getMotorPosition);
            rotatorTab.addNumber("Rotator Position - Degrees", () -> Units.rotationsToDegrees(getMotorPosition()));

            rotatorTab.addNumber("Rotator Setpoint - Rotations (Raw)", this::getTargetPosition);
            rotatorTab.addNumber("Rotator Setpoint - Degrees", () -> Units.rotationsToDegrees(getTargetPosition()));

            rotatorTab.addBoolean("Rotator At Target", this::isMotorAtTarget);

            rotatorTab.addNumber("Motor 1 Supply Current", () -> motor1SupplyCurrent.getValueAsDouble());
            rotatorTab.addNumber("Motor 2 Supply Current", () -> motor2SupplyCurrent.getValueAsDouble());
        }
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

        // ROTATOR_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        // ROTATOR_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimit = 30;
        // ROTATOR_MOTOR_CONFIG.CurrentLimits.SupplyCurrentThreshold = 40;
        // ROTATOR_MOTOR_CONFIG.CurrentLimits.SupplyTimeThreshold = 0.5;

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

    private void setDutyCycle(double dutyCycle) {
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
        return rotatorPosition.getValue();
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

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
                rotatorPosition,
                motor1SupplyCurrent,
                motor2SupplyCurrent);
        SmartDashboard.putNumber("rotator", Units.rotationsToDegrees(getMotorPosition()));
        SmartDashboard.putNumber("rotator target", Units.rotationsToDegrees(getTargetPosition()));

        if (Units.rotationsToDegrees(getMotorPosition()) >= 90) {
            rotatorState = RotatorState.OUTSIDE_UPPER_LIMIT;
        }

        else {
            rotatorState = RotatorState.INSIDE_LIMIT;
        }
    }
}
