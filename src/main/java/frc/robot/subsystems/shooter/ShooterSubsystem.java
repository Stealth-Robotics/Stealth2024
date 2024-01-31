package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;

    private double kS = 0.0;
    private double kV = 0.0;
    private double kA = 0.0;

    private double kP = 0.0;
    private double kI = 0.0;
    private double kD = 0.0;

    private final MotionMagicVelocityVoltage leftMotionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0, 0,
            false, 0, 0, false, false, false);
            
    private final MotionMagicVelocityVoltage rightMotionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0, 0,
            false, 0, 0, false, false, false);

    // value is in rotations per second
    // TODO: TUNE THIS TO A REASONABLE VALUE
    private final double VEOLOCITY_TOLERANCE = 0;

    public ShooterSubsystem() {
        // TODO: find can ids
        leftMotor = new TalonFX(0);
        rightMotor = new TalonFX(0);

        TalonFXConfiguration LEFT_TALONFX_CONFIG = new TalonFXConfiguration();
        TalonFXConfiguration RIGHT_TALONFX_CONFIG = new TalonFXConfiguration();

        // TODO: Check On Robot
        

        LEFT_TALONFX_CONFIG.Slot0.kS = kS;
        LEFT_TALONFX_CONFIG.Slot0.kV = kV;
        LEFT_TALONFX_CONFIG.Slot0.kA = kA;

        LEFT_TALONFX_CONFIG.Slot0.kP = kP;
        LEFT_TALONFX_CONFIG.Slot0.kI = kI;
        LEFT_TALONFX_CONFIG.Slot0.kD = kD;

        RIGHT_TALONFX_CONFIG = LEFT_TALONFX_CONFIG;

        
        //TODO: CHECK DIRECTIONS
        LEFT_TALONFX_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        RIGHT_TALONFX_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        leftMotor.getConfigurator().apply(LEFT_TALONFX_CONFIG);
        rightMotor.getConfigurator().apply(RIGHT_TALONFX_CONFIG);

        // will remove when shooter is tested
        throw new UnsupportedOperationException();
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
        rightMotor.setControl(new CoastOut());
        leftMotor.setControl(new CoastOut());
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


    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("current kS", () -> {return this.kS;}, (double kS) -> {this.kS = kS;});
        builder.addDoubleProperty("current kV", () -> {return this.kV;}, (double kV) -> {this.kV = kV;});
        builder.addDoubleProperty("current kA", () -> {return this.kA;}, (double kA) -> {this.kA = kA;});
        builder.addDoubleProperty("current kP", () -> {return this.kP;}, (double kP) -> {this.kP = kP;});
        builder.addDoubleProperty("current kI", () -> {return this.kI;}, (double kI) -> {this.kI = kI;});
        builder.addDoubleProperty("current kD", () -> {return this.kD;}, (double kD) -> {this.kD = kD;});
    }
}
