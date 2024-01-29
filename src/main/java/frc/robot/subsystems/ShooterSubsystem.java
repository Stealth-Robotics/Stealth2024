package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    
    private final TalonFXConfiguration LEFT_MOTION_MAGIC_CONFIG = new TalonFXConfiguration();
    private final TalonFXConfiguration RIGHT_MOTION_MAGIC_CONFIG = new TalonFXConfiguration();

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


        LEFT_MOTION_MAGIC_CONFIG.Slot0.kS = kS;
        LEFT_MOTION_MAGIC_CONFIG.Slot0.kV = kV;
        LEFT_MOTION_MAGIC_CONFIG.Slot0.kA = kA;
        LEFT_MOTION_MAGIC_CONFIG.Slot0.kP = kP;
        LEFT_MOTION_MAGIC_CONFIG.Slot0.kI = kI;
        LEFT_MOTION_MAGIC_CONFIG.Slot0.kD = kD;

        RIGHT_MOTION_MAGIC_CONFIG.Slot0.kS = kS;
        RIGHT_MOTION_MAGIC_CONFIG.Slot0.kV = kV;
        RIGHT_MOTION_MAGIC_CONFIG.Slot0.kA = kA;
        RIGHT_MOTION_MAGIC_CONFIG.Slot0.kP = kP;
        RIGHT_MOTION_MAGIC_CONFIG.Slot0.kI = kI;
        RIGHT_MOTION_MAGIC_CONFIG.Slot0.kD = kD;


        //TODO: CHEKC ON ROBOT
        leftMotor.setInverted(false);
        rightMotor.setInverted(false);

        leftMotor.getConfigurator().apply(LEFT_MOTION_MAGIC_CONFIG);
        rightMotor.getConfigurator().apply(RIGHT_MOTION_MAGIC_CONFIG);



    }
    

    /**
     * sets the target velocity for the left motor in rotations per second
     * @param velocity in rotations per second
     */
    public void setLeftVelocity(double velocity){
        
        LEFT_MOTION_MAGIC_CONFIG.MotionMagic.MotionMagicCruiseVelocity = velocity;
        //TODO: FIND THESE VALUES
        LEFT_MOTION_MAGIC_CONFIG.MotionMagic.MotionMagicAcceleration = 0;
        LEFT_MOTION_MAGIC_CONFIG.MotionMagic.MotionMagicJerk = 0;

        leftMotor.getConfigurator().apply(LEFT_MOTION_MAGIC_CONFIG);
    }

    /**
     * sets the target velocity for the right motor in rotations per second
     * @param velocity in rotations per second
     */
    public void setRightVelocity(double velocity){
        
        RIGHT_MOTION_MAGIC_CONFIG.MotionMagic.MotionMagicCruiseVelocity = velocity;
        //TODO: FIND THESE VALUES
        RIGHT_MOTION_MAGIC_CONFIG.MotionMagic.MotionMagicAcceleration = 0;
        RIGHT_MOTION_MAGIC_CONFIG.MotionMagic.MotionMagicJerk = 0;

        rightMotor.getConfigurator().apply(RIGHT_MOTION_MAGIC_CONFIG);
    }

    /**
     * returns the difference between the left motor's current velocity and the target velocity
     * @return velocity error in rotations per second
     */
    private double getLeftVelocityError(){
        return leftMotor.getVelocity().getValueAsDouble() - LEFT_MOTION_MAGIC_CONFIG.MotionMagic.MotionMagicCruiseVelocity;
    }

    /**
     * returns the difference between the right motor's current velocity and the target velocity
     * @return velocity error in rotations per second
     */
    private double getRightVelocityError(){
        return rightMotor.getVelocity().getValueAsDouble() - RIGHT_MOTION_MAGIC_CONFIG.MotionMagic.MotionMagicCruiseVelocity;
    }

    /**
     * returns if both motors are within the velocity tolerance
     * @return true if both motors are within the velocity tolerance
     */
    public boolean motorsAtTargetVelocity(){
        return Math.abs(getLeftVelocityError()) <= VEOLOCITY_TOLERANCE && Math.abs(getRightVelocityError()) <= VEOLOCITY_TOLERANCE;
    }
}
