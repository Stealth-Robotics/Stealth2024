public class LEDSubsystem extends SubsystemBase{

    private final int LedCount = 1;

    private final CANdle candle = new CANdle(RobotMap.Candle.CANDLE);

    private int toRed = 0;
    private int toGreen = 255;
    private int toBlue = 0;

    BooleanSupplier beamBreak;

    public LEDSubsystem (BooleanSupplier) {
        this.beamBreak = beamBreak;

        CANdleConfiguration config = new CANdleConfiguration();
        config.brightnessScalar = 9;
        config.disableWhenLOS = true;
        config.v5Enabled
    }
    
}
