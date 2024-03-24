package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {

    private final TalonFX indexerMotor;

    public IndexerSubsystem() {
        // TODO Get CAN ID
        indexerMotor = new TalonFX(-1);
    }

    public void setIndexerSpeed(double speed) {
        indexerMotor.set(speed);
    }

}
