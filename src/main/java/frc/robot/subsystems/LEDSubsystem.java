package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

    private final int LED_COUNT = 1;

    private Timer blinkTimer;

    private final CANdle candle = new CANdle(0);

    private LEDState currentState = LEDState.CoastMode_NotHomed;

    private Animation currentAnimation = null;

    private int currentRed = 0;
    private int currentGreen = 0;
    private int currentBlue = 0;

    private final BooleanSupplier beamBreak;
    private final BooleanSupplier isHomed;
    private final Supplier<NeutralModeValue> neutralMode;

    public enum LEDState {
        CoastMode_NotHomed,
        BrakeMode_NotHomed,

        CoastMode_Homed,
        BrakeMode_Homed,

        Idle,
        HasRing,
        WaitingForWantedState
    }

    public LEDSubsystem(BooleanSupplier beamBreak, BooleanSupplier isHomed, Supplier<NeutralModeValue> neutralMode) {
        this.beamBreak = beamBreak;
        this.isHomed = isHomed;
        this.neutralMode = neutralMode;

        blinkTimer = new Timer();
        blinkTimer.reset();

        CANdleConfiguration config = new CANdleConfiguration();
        config.brightnessScalar = 1.0;
        config.disableWhenLOS = true;
        config.v5Enabled = false;
        config.enableOptimizations = true;
        config.statusLedOffWhenActive = false;

        candle.configAllSettings(config);

        setWantedState(LEDState.Idle);
    }

    public void setWantedState(LEDState wantedState) {
        currentState = wantedState;
    }

    private void coastModeNotHomed() {
        currentAnimation = new StrobeAnimation(255, 0, 0, 0, 0.3, LED_COUNT);
    }

    private void brakeModeNotHomed() {
        currentAnimation = null;

        currentRed = 255;
        currentGreen = 0;
        currentBlue = 0;
    }

    private void coastModeHomed() {
        currentAnimation = new StrobeAnimation(0, 255, 0, 0, 0.3, LED_COUNT);
    }

    private void brakeModeHomed() {
        currentAnimation = null;

        currentRed = 0;
        currentGreen = 255;
        currentBlue = 0;
    }

    private void hasRing() {
        currentAnimation = new StrobeAnimation(234, 10, 142, 0, 0.3, LED_COUNT);
    }

    private void idle() {
        currentAnimation = new RainbowAnimation(1, 0.4, LED_COUNT);
    }

    @Override
    public void periodic() {

        if (DriverStation.isDisabled()) {
            if (isHomed.getAsBoolean()) {
                setWantedState(
                        neutralMode.get() == NeutralModeValue.Brake ? LEDState.BrakeMode_Homed
                                : LEDState.CoastMode_Homed);
            } else {
                setWantedState(neutralMode.get() == NeutralModeValue.Brake ? LEDState.BrakeMode_NotHomed
                        : LEDState.CoastMode_NotHomed);
            }
        }

        if (beamBreak.getAsBoolean()) {
            setWantedState(LEDState.HasRing);
        }

        switch (currentState) {
            case CoastMode_NotHomed:
                coastModeNotHomed();
                currentState = LEDState.WaitingForWantedState;
                break;
            case BrakeMode_NotHomed:
                brakeModeNotHomed();
                currentState = LEDState.WaitingForWantedState;
                break;
            case CoastMode_Homed:
                coastModeHomed();
                currentState = LEDState.WaitingForWantedState;
                break;
            case BrakeMode_Homed:
                brakeModeHomed();
                currentState = LEDState.WaitingForWantedState;
                break;
            case Idle:
                idle();
                currentState = LEDState.WaitingForWantedState;
                break;
            case HasRing:
                hasRing();
                blinkTimer.start();
                currentState = LEDState.WaitingForWantedState;
                break;
            case WaitingForWantedState:
                if (blinkTimer.hasElapsed(3)) {
                    blinkTimer.stop();
                    blinkTimer.reset();
                    currentState = LEDState.Idle;
                }
                break;
        }

        if (currentAnimation != null) {
            candle.animate(currentAnimation);
        } else {
            candle.setLEDs(currentRed, currentGreen, currentBlue, 0, 0, LED_COUNT);
        }
    }
}
