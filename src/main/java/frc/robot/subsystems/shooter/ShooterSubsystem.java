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

    // getters and setters for pid constants thru shuffleboard
    public double getkS() {
        return kS;
    }

    public double getkV() {
        return kV;
    }

    public double getkA() {
        return kA;
    }

    public double getkP() {
        return kP;
    }

    public double getkI() {
        return kI;
    }

    public double getkD() {
        return kD;
    }

    public void setkS(double kS) {
        this.kS = kS;
    }

    public void setkV(double kV) {
        this.kV = kV;
    }

    public void setkA(double kA) {
        this.kA = kA;
    }

    public void setkP(double kP) {
        this.kP = kP;
    }

    public void setkI(double kI) {
        this.kI = kI;
    }

    public void setkD(double kD) {
        this.kD = kD;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("current kS", this::getkS, this::setkS);
        builder.addDoubleProperty("current kV", this::getkV, this::setkV);
        builder.addDoubleProperty("current kA", this::getkA, this::setkA);
        builder.addDoubleProperty("current kP", this::getkP, this::setkP);
        builder.addDoubleProperty("current kI", this::getkI, this::setkI);
        builder.addDoubleProperty("current kD", this::getkD, this::setkD);
    }
}
