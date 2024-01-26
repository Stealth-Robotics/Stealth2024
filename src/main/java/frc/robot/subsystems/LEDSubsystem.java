package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

    private final int LedCount = 1;

    private Timer blinkTimer;

    private final CANdle candle = new CANdle(0);

    private LEDState currentState = LEDState.CoastMode;

    public enum LEDState {
        CoastMode,
        BrakeMode,
        Homed,
        Idle,
        HasRing
    }

    BooleanSupplier beamBreak;

    public LEDSubsystem(BooleanSupplier beamBreak) {
        this.beamBreak = beamBreak;
        blinkTimer = new Timer();

        CANdleConfiguration config = new CANdleConfiguration();
        config.brightnessScalar = 1.0;
        config.disableWhenLOS = true;
        config.v5Enabled = false;
        config.enableOptimizations = true;
        config.statusLedOffWhenActive = false;

        candle.configAllSettings(config);
    }

    public void setState(LEDState state)
    {
        currentState = state;
    }

    public void coastMode() {
        candle.animate(new StrobeAnimation(255, 255, 0, 0, 0.3, LedCount));
    }

    public void homed() {
        candle.setLEDs(0, 255, 0, 0, 0, LedCount);
    }

    public void hasRing() {
        candle.animate(new StrobeAnimation(234, 10, 142, 0, 0.3, LedCount));
    }

    public void idle() {
        candle.animate(new RainbowAnimation(1, 0.4, LedCount));
    }

    public void breakMode() {
        candle.animate(new StrobeAnimation(0, 255, 0, 0, 0.3, LedCount));
    }

    @Override
    public void periodic() {
        switch (currentState) {
            case CoastMode:
                coastMode();
                break;
            case BrakeMode:
                breakMode();
                break;
            case Homed:
                homed();
                break;
            case Idle:
                idle();
                break;
            case HasRing:
                hasRing();
                blinkTimer.reset();
                blinkTimer.start();
                if(blinkTimer.hasElapsed(3)){
                    setState(LEDState.Idle);
                    blinkTimer.stop();
                }
                break;

            default:
                idle();
        }
    }
}
