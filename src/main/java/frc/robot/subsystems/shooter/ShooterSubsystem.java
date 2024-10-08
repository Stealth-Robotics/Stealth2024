package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.function.DoubleSupplier;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;

    private final double kS = 0.195;
    private final double kV = 0.11;
    private final double kA = 0.0;

    private final double kP = 0.5;
    private final double kI = 0;
    private final double kD = 0.0;

    private final double MOTION_MAGIC_ACCELERATION = 400;
    private final double MOTION_MAGIC_JERK = 0.0;

    public final double SPIN_CONSTANT = 0.8;

    private final MotionMagicVelocityVoltage leftMotionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0, 0,
            false, 0, 0, false, false, false);

    private final MotionMagicVelocityVoltage rightMotionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0, 0,
            false, 0, 0, false, false, false);

    // value is in rotations per second
    // TODO: TUNE THIS TO A REASONABLE VALUE
    private final double VEOLOCITY_TOLERANCE = 1;

    TalonFXConfiguration LEFT_TALONFX_CONFIG = new TalonFXConfiguration();

    public ShooterSubsystem() {
        // TODO: find can ids
        leftMotor = new TalonFX(16);
        rightMotor = new TalonFX(17);

        applyConfigs();
    }

    private void applyConfigs() {
        LEFT_TALONFX_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        LEFT_TALONFX_CONFIG.Slot0.kS = kS;
        LEFT_TALONFX_CONFIG.Slot0.kV = kV;
        LEFT_TALONFX_CONFIG.Slot0.kA = kA;

        LEFT_TALONFX_CONFIG.Slot0.kP = kP;
        LEFT_TALONFX_CONFIG.Slot0.kI = kI;
        LEFT_TALONFX_CONFIG.Slot0.kD = kD;

        LEFT_TALONFX_CONFIG.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
        LEFT_TALONFX_CONFIG.MotionMagic.MotionMagicJerk = MOTION_MAGIC_JERK;

        LEFT_TALONFX_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        leftMotor.getConfigurator().apply(LEFT_TALONFX_CONFIG);
        rightMotor.getConfigurator().apply(LEFT_TALONFX_CONFIG);
    }

    /**
     * sets the target velocity for the left motor in rotations per second
     *
     * @param velocity
     *            in rotations per second
     */
    public void setLeftVelocity(double velocity) {
        leftMotionMagicVelocityVoltage.Velocity = velocity;
        leftMotor.setControl(leftMotionMagicVelocityVoltage);
    }

    /**
     * sets the target velocity for the right motor in rotations per second
     *
     * @param velocity
     *            in rotations per second
     */
    public void setRightVelocity(double velocity) {
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

    private void stopShooterMotors() {
        rightMotor.set(0);
        leftMotor.set(0);
    }

    public Command stopShooterMotorsCommand() {
        return this.runOnce(() -> stopShooterMotors())
                .andThen(new WaitUntilCommand(() -> (leftMotor.getVelocity().getValueAsDouble() <= VEOLOCITY_TOLERANCE
                        && rightMotor.getVelocity().getValueAsDouble() <= VEOLOCITY_TOLERANCE)));
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
     * @param rps
     *            doublesupplier for rotations per second to spin to
     * @return command that spins up the shooter to the target velocity based on
     *         distance from interpolation map
     */
    public Command spinToRps(DoubleSupplier rps) {

        return this.runOnce(() -> {
            setLeftVelocity(rps.getAsDouble());
            setRightVelocity(rps.getAsDouble() * SPIN_CONSTANT);
        }).andThen(new WaitUntilCommand(this::motorsAtTargetVelocity));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumberArray("shooter values",
                new Double[] { leftMotor.getVelocity().getValueAsDouble(), leftMotionMagicVelocityVoltage.Velocity });

        SmartDashboard.putNumber("velo", leftMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("velo target", leftMotionMagicVelocityVoltage.Velocity);
    }
}
