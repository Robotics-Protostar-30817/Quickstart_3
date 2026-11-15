package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import dev.nextftc.core.subsystems.Subsystem;

/** Continuous-rotation servo driving the floor intake ramp. */
public final class IntakeRamp implements Subsystem {
    public static final IntakeRamp INSTANCE = new IntakeRamp();
    public static double INTAKE_POWER = 0.5;
    private CRServo servo;
    private double power;
    private IntakeRamp() {}
    public void init(CRServo servo) {
        if (servo == null) throw new IllegalArgumentException("Ramp CRServo is null");
        this.servo = servo; stop();
    }
    public void setPower(double value) {
        if (servo == null) throw new IllegalStateException("IntakeRamp.init() required");
        if (!Double.isFinite(value)) throw new IllegalArgumentException("Invalid ramp power");
        power = Math.max(-1, Math.min(1, value)); servo.setPower(power);
    }
    public void intake() { setPower(INTAKE_POWER); }
    public void stop() { power = 0; if (servo != null) servo.setPower(0); }
    public double getPower() { return power; }
    @Override public void periodic() {}
}

