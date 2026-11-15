package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import dev.nextftc.core.subsystems.Subsystem;

/** Positional servo that raises/lowers the floor intake ramp. */
public final class IntakeRamp implements Subsystem {
    public static final IntakeRamp INSTANCE = new IntakeRamp();

    // PLACEHOLDERS: calibrate with linkage disconnected to avoid mechanical damage.
    public static double RAISED_POSITION = 0.50;
    public static double LOWERED_POSITION = 0.60;

    private Servo servo;
    private boolean lowered;

    private IntakeRamp() {}

    public void init(Servo rampServo) {
        if (rampServo == null) {
            throw new IllegalArgumentException("Intake ramp servo is null");
        }
        servo = rampServo;
        lowered = false;
        // Explicitly command a safe initial position.
        raise();
    }

    private void command(double position) {
        if (servo == null) {
            throw new IllegalStateException("IntakeRamp.init() required");
        }
        if (!Double.isFinite(position) || position < 0.0 || position > 1.0) {
            throw new IllegalStateException("Invalid intake ramp servo position");
        }
        servo.setPosition(position);
    }

    public void lower() {
        command(LOWERED_POSITION);
        lowered = true;
    }

    public void raise() {
        command(RAISED_POSITION);
        lowered = false;
    }

    /** Compatibility with CompleteShooter.startCollecting(). */
    public void intake() { lower(); }

    /** Compatibility with CompleteShooter.stopCollecting(). */
    public void stop() { if (servo != null) raise(); }

    /** Returns the last commanded state, not a measured servo position. */
    public boolean isLowered() { return lowered; }

    /** Returns the last commanded position (FTC Servo does not report physical feedback). */
    public double getPosition() {
        if (servo == null) throw new IllegalStateException("IntakeRamp.init() required");
        return lowered ? LOWERED_POSITION : RAISED_POSITION;
    }

    @Override public void periodic() {}
}
