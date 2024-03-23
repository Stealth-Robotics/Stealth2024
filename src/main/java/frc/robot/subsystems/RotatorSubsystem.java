package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

public class RotatorSubsystem extends SubsystemBase {

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

    private final double kS = 0.0;
    private final double kV = 14.8;
    private final double kG = 0.3;

    private final double kP = 30;
    private final double kI = 10;
    private final double kD = 1;

    private final DigitalInput homeButton = new DigitalInput(0);
    private final DigitalInput toggleMotorModeButton = new DigitalInput(1);

    private NeutralModeValue motorMode = NeutralModeValue.Coast;
    private boolean isHomed = false;

    // tolerance in radians
    // TODO: TUNE THIS
    // this is a tolerance of 1 degree
    private final double kTOLERANCE = Units.degreesToRotations(0.75);

    private final double MOTION_MAGIC_JERK = 6;
    private final double MOTION_MAGIC_ACCELERATION = 2;
    private final double MOTION_MAGIC_CRUISE_VELOCITY = 0.5;

    private final TalonFXConfiguration ROTATOR_MOTOR_CONFIG = new TalonFXConfiguration();

    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    private final GenericEntry ampOutIntake;

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
                DriverStation::isDisabled);

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
                DriverStation::isDisabled);

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

    private boolean isMotorAtTarget() {
        return Math.abs(getMotorPosition() - getTargetPosition()) <= kTOLERANCE;
    }

    private void setMotorTargetPosition(double rotations) {
        motionMagicVoltage.Position = rotations;
        rotatorMotorOne.setControl(motionMagicVoltage);
    }

    public Command rotateToPositionCommand(DoubleSupplier rotations) {
        return Commands.runOnce(() -> setMotorTargetPosition(rotations.getAsDouble()), this)
                .andThen(new WaitUntilCommand(this::isMotorAtTarget));
    }

    public Command rotateWhileDrivingCommand(DoubleSupplier rotations){
        return Commands.run(() -> this.setMotorTargetPosition(rotations.getAsDouble()));
    }

    public BooleanSupplier getAmpOutIntake() {
        return () -> ampOutIntake.getBoolean(false);
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("rotator", Units.rotationsToDegrees(getMotorPosition()));
        SmartDashboard.putNumber("rotator target", Units.rotationsToDegrees(getTargetPosition()));

    }
}
