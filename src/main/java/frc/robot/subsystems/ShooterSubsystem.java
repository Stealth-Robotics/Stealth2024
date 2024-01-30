package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    
    private final TalonFXConfiguration MOTION_MAGIC_CONFIG = new TalonFXConfiguration();

    private final MotionMagicVelocityVoltage LEFT_MOTION_MAGIC_VELOCITY_VOLTAGE = new MotionMagicVelocityVoltage(0);
    private final MotionMagicVelocityVoltage RIGHT_MOTION_MAGIC_VELOCITY_VOLTAGE = new MotionMagicVelocityVoltage(0);

    private final double kS = 0.0;
    private final double kV = 0.0;
    private final double kA = 0.0;
    private final double kP = 0.0;
    private final double kI = 0.0;
    private final double kD = 0.0;

    //value is in rotations per second
    //TODO: TUNE THIS TO A REASONABLE VALUE
    private final double VEOLOCITY_TOLERANCE = 0;
    

    public ShooterSubsystem(){
        //TODO: find can ids
        leftMotor = new TalonFX(0);
        rightMotor = new TalonFX(0);


        MOTION_MAGIC_CONFIG.Slot0.kS = kS;
        MOTION_MAGIC_CONFIG.Slot0.kV = kV;
        MOTION_MAGIC_CONFIG.Slot0.kA = kA;
        MOTION_MAGIC_CONFIG.Slot0.kP = kP;
        MOTION_MAGIC_CONFIG.Slot0.kI = kI;
        MOTION_MAGIC_CONFIG.Slot0.kD = kD;

        //TODO: FIND THESE VALUES
        MOTION_MAGIC_CONFIG.MotionMagic.MotionMagicCruiseVelocity = 0;
        MOTION_MAGIC_CONFIG.MotionMagic.MotionMagicAcceleration = 0;
        MOTION_MAGIC_CONFIG.MotionMagic.MotionMagicJerk = 0;



        //TODO: CHEKC ON ROBOT
        leftMotor.setInverted(false);
        rightMotor.setInverted(false);

        leftMotor.getConfigurator().apply(MOTION_MAGIC_CONFIG);
        rightMotor.getConfigurator().apply(MOTION_MAGIC_CONFIG);



    }
    

    /**
     * sets the target velocity for the left motor in rotations per second
     * @param velocity in rotations per second
     */
    public void setLeftVelocity(double velocity){
        LEFT_MOTION_MAGIC_VELOCITY_VOLTAGE.Velocity = velocity;
        leftMotor.setControl(LEFT_MOTION_MAGIC_VELOCITY_VOLTAGE);
    }

    /**
     * sets the target velocity for the right motor in rotations per second
     * @param velocity in rotations per second
     */
    public void setRightVelocity(double velocity){
        RIGHT_MOTION_MAGIC_VELOCITY_VOLTAGE.Velocity = velocity;
        rightMotor.setControl(RIGHT_MOTION_MAGIC_VELOCITY_VOLTAGE);
    }

    /**
     * returns the difference between the left motor's current velocity and the target velocity
     * @return velocity error in rotations per second
     */
    private double getLeftVelocityError(){
        return leftMotor.getVelocity().getValueAsDouble() - LEFT_MOTION_MAGIC_VELOCITY_VOLTAGE.Velocity;
    }

    /**
     * returns the difference between the right motor's current velocity and the target velocity
     * @return velocity error in rotations per second
     */
    private double getRightVelocityError(){
        return rightMotor.getVelocity().getValueAsDouble() - RIGHT_MOTION_MAGIC_VELOCITY_VOLTAGE.Velocity;
    }

    public void stopShooterMotors(){
        rightMotor.setControl(new CoastOut());
        leftMotor.setControl(new CoastOut());
    }
    

    /**
     * returns if both motors are within the velocity tolerance
     * @return true if both motors are within the velocity tolerance
     */
    public boolean motorsAtTargetVelocity(){
        return Math.abs(getLeftVelocityError()) <= VEOLOCITY_TOLERANCE && Math.abs(getRightVelocityError()) <= VEOLOCITY_TOLERANCE;
    }
}
