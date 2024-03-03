package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;

    private double kS = 0.195;
    private double kV = 0.11;
    private double kA = 0.0;

    private double kP = 0.5;
    private double kI = 0;
    private double kD = 0.0;

    private double MOTION_MAGIC_ACCELERATION = 100;
    private double MOTION_MAGIC_JERK = 0.0;

    public final double SPIN_CONSTANT = 0.8;

    private final MotionMagicVelocityVoltage leftMotionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0, 0,
            false, 0, 0, false, false, false);

    private final MotionMagicVelocityVoltage rightMotionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0, 0,
            false, 0, 0, false, false, false);

    // value is in rotations per second
    // TODO: TUNE THIS TO A REASONABLE VALUE
    private final double VEOLOCITY_TOLERANCE = 1;

    TalonFXConfiguration LEFT_TALONFX_CONFIG = new TalonFXConfiguration();
    TalonFXConfiguration RIGHT_TALONFX_CONFIG = new TalonFXConfiguration();

    public ShooterSubsystem() {
        // TODO: find can ids
        leftMotor = new TalonFX(16);
        rightMotor = new TalonFX(17);

        applyConfigs();

        if (Constants.TUNING_MODE) {
            ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");

            shooterTab.add(leftMotor);
            shooterTab.add(rightMotor);

            shooterTab.addNumber("Left Motor Velocity", this::getLeftVelocity);
            shooterTab.addNumber("Left Error", this::getLeftVelocityError);

            shooterTab.addNumber("Right Velocity", this::getRightVelocity);
            shooterTab.addNumber("Right Velocity Error", this::getRightVelocityError);

            shooterTab.addBoolean("At Target", this::motorsAtTargetVelocity);

            shooterTab.addDouble("Left Supply Current", this::getLeftSupplyCurrent);
            shooterTab.addDouble("Right Supply Current", this::getRightSupplyCurrent);
        }
    }

    private void applyConfigs() {
        // TODO: check everything
        LEFT_TALONFX_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        LEFT_TALONFX_CONFIG.Slot0.kS = kS;
        LEFT_TALONFX_CONFIG.Slot0.kV = kV;
        LEFT_TALONFX_CONFIG.Slot0.kA = kA;

        LEFT_TALONFX_CONFIG.Slot0.kP = kP;
        LEFT_TALONFX_CONFIG.Slot0.kI = kI;
        LEFT_TALONFX_CONFIG.Slot0.kD = kD;

        LEFT_TALONFX_CONFIG.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
        LEFT_TALONFX_CONFIG.MotionMagic.MotionMagicJerk = MOTION_MAGIC_JERK;

        // LEFT_TALONFX_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        // LEFT_TALONFX_CONFIG.CurrentLimits.SupplyCurrentLimit = 30;
        // LEFT_TALONFX_CONFIG.CurrentLimits.SupplyCurrentThreshold = 40;
        // LEFT_TALONFX_CONFIG.CurrentLimits.SupplyTimeThreshold = 1.0;

        RIGHT_TALONFX_CONFIG = LEFT_TALONFX_CONFIG;

        LEFT_TALONFX_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        RIGHT_TALONFX_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        leftMotor.getConfigurator().apply(LEFT_TALONFX_CONFIG);
        rightMotor.getConfigurator().apply(RIGHT_TALONFX_CONFIG);
    }

    private double getLeftSupplyCurrent()
    {
        return leftMotor.getSupplyCurrent().getValueAsDouble();
    }

    private double getRightSupplyCurrent()
    {
        return rightMotor.getSupplyCurrent().getValueAsDouble();
    }

    /**
     * sets the target velocity for the left motor in rotations per second
     * 
     * @param velocity in rotations per second
     */
    public void setLeftVelocity(double velocity) {
        leftMotionMagicVelocityVoltage.Velocity = velocity;
        leftMotor.setControl(leftMotionMagicVelocityVoltage);
    }

    /**
     * sets the target velocity for the right motor in rotations per second
     * 
     * @param velocity in rotations per second
     */
    public void setRightVelocity(double velocity) {
        rightMotionMagicVelocityVoltage.Velocity = velocity;
        rightMotor.setControl(rightMotionMagicVelocityVoltage);
    }

    private double getLeftVelocity()
    {
        return leftMotor.getVelocity().getValueAsDouble();
    }

    private double getRightVelocity()
    {
        return rightMotor.getVelocity().getValueAsDouble();
    }

    /**
     * returns the difference between the left motor's current velocity and the
     * target velocity
     * 
     * @return velocity error in rotations per second
     */
    private double getLeftVelocityError() {
        return leftMotor.getVelocity().getValueAsDouble() - leftMotionMagicVelocityVoltage.Velocity;
    }

    /**
     * returns the difference between the right motor's current velocity and the
     * target velocity
     * 
     * @return velocity error in rotations per second
     */
    private double getRightVelocityError() {
        return rightMotor.getVelocity().getValueAsDouble() - rightMotionMagicVelocityVoltage.Velocity;
    }

    public void stopShooterMotors() {
        rightMotor.set(0);
        leftMotor.set(0);
    }

    /**
     * returns if both motors are within the velocity tolerance
     * 
     * @return true if both motors are within the velocity tolerance
     */
    public boolean motorsAtTargetVelocity() {
        return Math.abs(getLeftVelocityError()) <= VEOLOCITY_TOLERANCE
                && Math.abs(getRightVelocityError()) <= VEOLOCITY_TOLERANCE;
    }

    public void setShooterDutyCycle(double dutyCycle){
        leftMotor.set(dutyCycle);
        rightMotor.set(dutyCycle);

    }

    /**
     * returns a command that spins up the shooter to the target velocity based on
     * distance from goal
     * 
     * @param distance in feet from goal
     * @return command that spins up the shooter to the target velocity based on
     *         distance from interpolation map
     */
    public Command spinToRps(double rps) {

        return new InstantCommand(() -> {
            setLeftVelocity(rps);
            setRightVelocity(rps * SPIN_CONSTANT);
        }, this).andThen(new WaitUntilCommand(this::motorsAtTargetVelocity));
    }

    public Command spinToRps(double rps, double spinConstant) {

        return new InstantCommand(() -> {
            setLeftVelocity(rps);
            setRightVelocity(rps * spinConstant);
        }, this).andThen(new WaitUntilCommand(this::motorsAtTargetVelocity));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumberArray("shooter values",
                new Double[] { leftMotor.getVelocity().getValueAsDouble(), leftMotionMagicVelocityVoltage.Velocity });

    }
}
