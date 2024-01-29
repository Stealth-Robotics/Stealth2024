package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    
    private final MotionMagicConfigs LEFT_MOTION_MAGIC_CONFIG = new TalonFXConfiguration().MotionMagic;
    private final MotionMagicConfigs RIGHT_MOTION_MAGIC_CONFIG = new TalonFXConfiguration().MotionMagic;

    //value is in rotations per second
    //TODO: TUNE THIS TO A REASONABLE VALUE
    private final double VEOLOCITY_TOLERANCE = 0;
    

    public ShooterSubsystem(){
        leftMotor = new TalonFX(1);
        rightMotor = new TalonFX(2);

        

    }
    

    public void setLeftVelocity(double velocity){
        
        LEFT_MOTION_MAGIC_CONFIG.MotionMagicCruiseVelocity = velocity;
        //TODO: FIND THESE VALUES
        LEFT_MOTION_MAGIC_CONFIG.MotionMagicAcceleration = 0;
        LEFT_MOTION_MAGIC_CONFIG.MotionMagicJerk = 0;

        leftMotor.getConfigurator().apply(LEFT_MOTION_MAGIC_CONFIG);
    }

    public void setRightVelocity(double velocity){
        
        RIGHT_MOTION_MAGIC_CONFIG.MotionMagicCruiseVelocity = velocity;
        //TODO: FIND THESE VALUES
        RIGHT_MOTION_MAGIC_CONFIG.MotionMagicAcceleration = 0;
        RIGHT_MOTION_MAGIC_CONFIG.MotionMagicJerk = 0;

        rightMotor.getConfigurator().apply(RIGHT_MOTION_MAGIC_CONFIG);
    }

    private double getLeftVelocityError(){
        return leftMotor.getVelocity().getValueAsDouble() - LEFT_MOTION_MAGIC_CONFIG.MotionMagicCruiseVelocity;
    }
    
    private double getRightVelocityError(){
        return rightMotor.getVelocity().getValueAsDouble() - RIGHT_MOTION_MAGIC_CONFIG.MotionMagicCruiseVelocity;
    }

    public boolean motorsAtTargetVelocity(){
        return Math.abs(getLeftVelocityError()) <= VEOLOCITY_TOLERANCE && Math.abs(getRightVelocityError()) <= VEOLOCITY_TOLERANCE;
    }
}
