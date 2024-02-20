package frc.robot;

public class Constants {
    public static final Mode currentMode = Mode.REAL;
    public static final boolean tuningMode = true;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

}
