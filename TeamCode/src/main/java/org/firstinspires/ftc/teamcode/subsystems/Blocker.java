package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import dev.nextftc.core.subsystems.Subsystem;

/** Independent safety gate preventing POLLEN from reaching the flywheel. */
public final class Blocker implements Subsystem {
    public static final Blocker INSTANCE = new Blocker();
    // PLACEHOLDERS: measure safe positions with mechanism disconnected first.
    public static double CLOSED_POSITION = 0.50;
    public static double OPEN_POSITION = 0.60;
    private Servo servo;
    private boolean open;
    private Blocker() {}
    public void init(Servo servo) {
        if (servo == null) throw new IllegalArgumentException("Blocker servo is null");
        this.servo = servo; close();
    }
    private void set(double position) {
        if (servo == null) throw new IllegalStateException("Blocker.init() required");
        if (!Double.isFinite(position) || position < 0 || position > 1)
            throw new IllegalStateException("Invalid blocker calibration");
        servo.setPosition(position);
    }
    public void open() { set(OPEN_POSITION); open = true; }
    public void close() { set(CLOSED_POSITION); open = false; }
    public boolean isOpen() { return open; }
    @Override public void periodic() {}
}

