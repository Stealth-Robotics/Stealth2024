package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.BetterInstantCommand;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;

    private double kS = 0.195;
    private double kV = 0.11;
    private double kA = 0.0;

    private double kP = 0.6;
    private double kI = 0;
    private double kD = 0.0;

    private double MOTION_MAGIC_ACCELERATION = 100;
    private double MOTION_MAGIC_JERK = 0.0;

    private final double SPIN_CONSTANT = 0.5;

    private final StatusSignal<Double> leftShooterVelocity;
    private final StatusSignal<Double> rightShooterVelocity; 

    private final MotionMagicVelocityVoltage leftMotionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0, 0,
            false, 0, 0, false, false, false);

    private final MotionMagicVelocityVoltage rightMotionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0, 0,
            false, 0, 0, false, false, false);

    // value is in rotations per second
    // TODO: TUNE THIS TO A REASONABLE VALUE
    private final double VEOLOCITY_TOLERANCE = 5;

    TalonFXConfiguration LEFT_TALONFX_CONFIG = new TalonFXConfiguration();
    TalonFXConfiguration RIGHT_TALONFX_CONFIG = new TalonFXConfiguration();

    public ShooterSubsystem() {
        // TODO: find can ids
        leftMotor = new TalonFX(16);
        rightMotor = new TalonFX(17);

        applyConfigs();

        leftShooterVelocity = leftMotor.getVelocity();
        leftShooterVelocity.setUpdateFrequency(500);

        rightShooterVelocity = rightMotor.getVelocity();
        rightShooterVelocity.setUpdateFrequency(500);

        leftMotor.optimizeBusUtilization();
        rightMotor.optimizeBusUtilization();
    }

    private void applyConfigs() {
        // TODO: check everything
        LEFT_TALONFX_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        LEFT_TALONFX_CONFIG.Slot0.kS = kS;
        LEFT_TALONFX_CONFIG.Slot0.kV = kV;
        LEFT_TALONFX_CONFIG.Slot0.kA = kA;

        LEFT_TALONFX_CONFIG.Slot0.kP = kP;
        LEFT_TALONFX_CONFIG.Slot0.kI = kI;
        LEFT_TALONFX_CONFIG.Slot0.kD = kD;

        LEFT_TALONFX_CONFIG.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
        LEFT_TALONFX_CONFIG.MotionMagic.MotionMagicJerk = MOTION_MAGIC_JERK;

        RIGHT_TALONFX_CONFIG = LEFT_TALONFX_CONFIG;

        LEFT_TALONFX_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        RIGHT_TALONFX_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        leftMotor.getConfigurator().apply(LEFT_TALONFX_CONFIG);
        rightMotor.getConfigurator().apply(RIGHT_TALONFX_CONFIG);
    }

    /**
     * sets the target velocity for the left motor in rotations per second
     * 
     * @param velocity in rotations per second
     */
    private void setLeftVelocity(double velocity) {
        leftMotionMagicVelocityVoltage.Velocity = velocity;
        leftMotor.setControl(leftMotionMagicVelocityVoltage);
    }

    /**
     * sets the target velocity for the right motor in rotations per second
     * 
     * @param velocity in rotations per second
     */
    private void setRightVelocity(double velocity) {
        rightMotionMagicVelocityVoltage.Velocity = velocity;
        rightMotor.setControl(rightMotionMagicVelocityVoltage);
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
        leftMotionMagicVelocityVoltage.Velocity = 0;
        rightMotionMagicVelocityVoltage.Velocity = 0;
        rightMotor.setControl(new CoastOut());
        leftMotor.setControl(new CoastOut());
    }

    /**
     * returns if both motors are within the velocity tolerance
     * 
     * @return true if both motors are within the velocity tolerance
     */
    private boolean motorsAtTargetVelocity() {
        return Math.abs(getLeftVelocityError()) <= VEOLOCITY_TOLERANCE
                && Math.abs(getRightVelocityError()) <= VEOLOCITY_TOLERANCE;
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

        return new BetterInstantCommand(
            () -> {
                setLeftVelocity(rps);
                setRightVelocity(rps * SPIN_CONSTANT);
            }, this
        ).until(() -> this.motorsAtTargetVelocity());
    }

    @Override
    public void periodic() {

        BaseStatusSignal.refreshAll(
            leftShooterVelocity,
            rightShooterVelocity
        );

        SmartDashboard.putNumberArray("shooter values",
                new Double[] { leftMotor.getVelocity().getValueAsDouble(), leftMotionMagicVelocityVoltage.Velocity });

    }
}
