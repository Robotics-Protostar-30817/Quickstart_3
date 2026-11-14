package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import dev.nextftc.core.subsystems.Subsystem;

/**
 * Single positional servo for shooter hood adjustment.
 *
 * The servo must be programmed for POSITION mode, not continuous rotation.
 * Position values are servo commands, NOT measured shooter launch angles.
 * Calibrate the linkage and safe range before enabling full travel.
 */
public final class Hood implements Subsystem {
    public static final Hood INSTANCE = new Hood();

    // Conservative initial command window; verify it is safe on your mechanism.
    public static double MIN_POSITION = 0.45;
    public static double MAX_POSITION = 0.55;
    public static double TEST_STEP = 0.01;

    private Servo servo;
    private double commandedPosition = 0.5;
    private boolean hasCommand = false;

    private Hood() { }

    public void init(Servo hoodServo) {
        if (hoodServo == null) throw new IllegalArgumentException("hoodServo is null");
        servo = hoodServo;
        // Do not move the mechanism automatically at initialization.
        commandedPosition = Range.clip(servo.getPosition(), 0.0, 1.0);
        hasCommand = false;
    }

    private void requireInitialized() {
        if (servo == null) throw new IllegalStateException("Hood.init() not called");
    }

    public void setPosition(double position) {
        requireInitialized();
        if (MIN_POSITION > MAX_POSITION) throw new IllegalStateException("Invalid hood position limits");
        commandedPosition = Range.clip(position, MIN_POSITION, MAX_POSITION);
        servo.setPosition(commandedPosition);
        hasCommand = true;
    }

    public void adjustPosition(double delta) {
        setPosition(commandedPosition + delta);
    }

    /** Last requested position; not independent position feedback. */
    public double getPosition() {
        requireInitialized();
        return commandedPosition;
    }

    public boolean hasCommand() { return hasCommand; }

    /** Keep the servo at its most recently commanded position. */
    public void hold() {
        requireInitialized();
        if (hasCommand) servo.setPosition(commandedPosition);
    }

    @Override
    public void periodic() {
        // Standard positional servo handles position control internally.
    }
}
