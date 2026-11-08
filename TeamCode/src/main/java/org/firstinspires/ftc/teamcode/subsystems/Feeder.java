package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import dev.nextftc.core.subsystems.Subsystem;

/**
 * Feeder motor subsystem. Intended for one goBILDA Yellow Jacket 1150 RPM motor.
 * This class does not count POLLEN; counting requires a sensor or calibrated indexing.
 */
public final class Feeder implements Subsystem {
    public static final Feeder INSTANCE = new Feeder();
    public static final double RATED_NO_LOAD_RPM = 1620.0;

    private DcMotorEx motor;
    private double commandedPower = 0.0;

    private Feeder() { }

    public void init(DcMotorEx feederMotor) {
        if (feederMotor == null) throw new IllegalArgumentException("Feeder motor is null");
        motor = feederMotor;
        motor.setPower(0.0);
        // Verify that FORWARD feeds POLLEN toward the flywheel.
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        commandedPower = 0.0;
    }

    public void setPower(double power) {
        if (motor == null) throw new IllegalStateException("Call Feeder.init() first");
        if (!Double.isFinite(power)) throw new IllegalArgumentException("Invalid feeder power");
        commandedPower = Math.max(-1.0, Math.min(1.0, power));
        motor.setPower(commandedPower);
    }

    public void start(double power) {
        if (power <= 0.0) throw new IllegalArgumentException("Feeder start power must be positive");
        setPower(power);
    }

    public void stop() {
        commandedPower = 0.0;
        if (motor != null) motor.setPower(0.0);
    }

    public boolean isRunning() { return commandedPower != 0.0; }
    public double getCommandedPower() { return commandedPower; }
    public double getEncoderVelocity() { return motor == null ? 0.0 : motor.getVelocity(); }

    @Override public void periodic() {
        // No periodic work required for open-loop feeder power control.
    }
}

