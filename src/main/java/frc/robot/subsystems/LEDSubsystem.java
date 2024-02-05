package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

    private final int LED_COUNT = 8;

    private Timer blinkTimer;

    private final CANdle candle = new CANdle(0);

    private boolean isAnimating = false;

    private int currentRed = 0;
    private int currentGreen = 0;
    private int currentBlue = 0;

    public LEDSubsystem() {

        blinkTimer = new Timer();
        blinkTimer.reset();

        CANdleConfiguration config = new CANdleConfiguration();
        config.brightnessScalar = 1.0;
        config.disableWhenLOS = true;
        config.v5Enabled = false;
        config.enableOptimizations = true;
        config.statusLedOffWhenActive = false;

        candle.configAllSettings(config);
    }

    private void setRGB(int red, int green, int blue) {
        isAnimating = false;
        currentRed = red;
        currentGreen = green;
        currentBlue = blue;
    }

    private void animate(Animation animation) {
        isAnimating = true;
        candle.animate(animation);
    }

    public void coastModeNotHomed() {
        animate(new StrobeAnimation(255, 0, 0, 0, 0.3, LED_COUNT));
    }

    public void brakeModeNotHomed() {
        setRGB(255, 0, 0);
    }

    public void coastModeHomed() {
        animate(new StrobeAnimation(0, 255, 0, 0, 0.3, LED_COUNT));
    }

    public void brakeModeHomed() {
        setRGB(0, 255, 0);
    }

    public void hasRing() {
        animate(new StrobeAnimation(234, 10, 142, 0, 0.3, LED_COUNT));        
    }

    public void idle() {
        animate(new RainbowAnimation(1, 0.4, LED_COUNT));
    }

    @Override
    public void periodic() {
        if (!isAnimating) {
            candle.setLEDs(currentRed, currentGreen, currentBlue, 0, 0, LED_COUNT);
        }
    }
}
