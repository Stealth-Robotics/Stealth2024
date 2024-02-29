package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LEDSubsystem extends SubsystemBase {

    private final int LED_COUNT = 74;

    private final CANdle candle = new CANdle(21);

    int currentRed = 0;
    int currentGreen = 0;
    int currentBlue = 0;

    boolean isAnimating = false;

    BooleanSupplier isHomeBooleanSupplier;
    BooleanSupplier isBrakeModeSupplier;

    BooleanSupplier hasRingSupplier;

    public LEDSubsystem(BooleanSupplier isHomedSupplier, BooleanSupplier isBrakeModeSupplier, BooleanSupplier hasRingSupplier) {

        this.isHomeBooleanSupplier = isHomedSupplier;
        this.isBrakeModeSupplier = isBrakeModeSupplier;
        this.hasRingSupplier = hasRingSupplier;

        CANdleConfiguration config = new CANdleConfiguration();
        config.brightnessScalar = 1.0;
        config.disableWhenLOS = true;
        config.v5Enabled = true;
        config.vBatOutputMode = VBatOutputMode.Off;
        config.enableOptimizations = true;
        config.statusLedOffWhenActive = false;

        candle.configAllSettings(config);

        updateDisabledLEDs();
    }

    private void setRGB(int red, int green, int blue) {
        isAnimating = false;

        candle.clearAnimation(0);

        currentRed = red;
        currentGreen = green;
        currentBlue = blue;
    }

    private void animate(Animation animation) {
        isAnimating = true;

        candle.clearAnimation(0);

        candle.animate(animation, 0);
    }

    private void coastModeNotHomed() {
        animate(new SingleFadeAnimation(255, 0, 0, 0, 0.85, LED_COUNT));
    }

    private void brakeModeNotHomed() {
        setRGB(255, 0, 0);
    }

    private void coastModeHomed() {
        animate(new SingleFadeAnimation(0, 255, 0, 0, 0.85, LED_COUNT));
    }

    private void brakeModeHomed() {
        setRGB(0, 255, 0);
    }

    private void gainedRing() {
        // Change to SingleFadeAnimation if StrobeAnimation is too bright
        animate(new StrobeAnimation(255, 0, 100, 0, 0.2, LED_COUNT));
    }

    private void hasRing() {
        setRGB(0, 255, 0);
    }

    public void idle() {
        animate(new RainbowAnimation(1, 0.6, LED_COUNT, true, 0));
    }

    public void updateDisabledLEDs() {
        if (DriverStation.isDisabled()) {
            if (isHomeBooleanSupplier.getAsBoolean()) {
                if (isBrakeModeSupplier.getAsBoolean()) {
                    brakeModeHomed();
                } else {
                    coastModeHomed();
                }
            } else {
                if (isBrakeModeSupplier.getAsBoolean()) {
                    brakeModeNotHomed();
                } else {
                    coastModeNotHomed();
                }
            }
        }
    }

    public Command updateDisabledLEDsCommand() {
        return new InstantCommand(
            () -> updateDisabledLEDs(), 
            this
        ).ignoringDisable(true);
    }

    public Command blinkForRingCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(
                () -> gainedRing(), 
                this
            ),
            new WaitCommand(2.5),
            idleCommand());
    }

    public Command idleCommand()
    {
        return new InstantCommand(() -> idle(), this);
    }

    @Override
    public void periodic() {
        if (!isAnimating) {
            candle.setLEDs(currentRed, currentGreen, currentBlue, 0, 0, LED_COUNT);
        }
    }
}
