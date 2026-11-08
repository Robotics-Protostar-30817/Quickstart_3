package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import dev.nextftc.core.subsystems.Subsystem;

/**
 * Encoder-relative turret positioning. The position at init() is zero;
 * this is NOT an absolute/home reference.
 *
 * Set MOTOR_REVS_PER_TURRET_REV to the actual external gear ratio.
 * Set safe angle limits for your mechanical installation BEFORE testing.
 */
public final class Turret implements Subsystem {
    public static final Turret INSTANCE = new Turret();

    public static final double RATED_MOTOR_RPM = 435.0;
    public static double MOTOR_REVS_PER_TURRET_REV = 1.0; // TODO: measure gearing
    public static double MIN_ANGLE_DEG = -30.0; // conservative placeholders
    public static double MAX_ANGLE_DEG = 30.0;
    public static double MAX_POWER = 0.20;
    public static double KP = 0.012; // power per degree; tune on robot
    public static double ANGLE_TOLERANCE_DEG = 1.5;

    private DcMotorEx motor;
    private int zeroTicks;
    private double targetAngleDeg;
    private boolean positionMode;
    private double manualPower;

    private Turret() {}

    public void init(DcMotorEx turretMotor) {
        motor = turretMotor;
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0);
        zeroTicks = motor.getCurrentPosition();
        targetAngleDeg = 0;
        manualPower = 0;
        positionMode = false;
    }

    private void requireInitialized() {
        if (motor == null) throw new IllegalStateException("Turret.init() not called");
    }

    public double getTicksPerMotorRevolution() {
        requireInitialized();
        return motor.getMotorType().getTicksPerRev();
    }

    public double getAngleDegrees() {
        requireInitialized();
        double ticksPerTurretRev = getTicksPerMotorRevolution() * MOTOR_REVS_PER_TURRET_REV;
        if (ticksPerTurretRev <= 0) throw new IllegalStateException("Invalid turret gearing/encoder configuration");
        return (motor.getCurrentPosition() - zeroTicks) * 360.0 / ticksPerTurretRev;
    }

    public int getEncoderTicks() {
        requireInitialized();
        return motor.getCurrentPosition();
    }

    public void setTargetAngle(double degrees) {
        requireInitialized();
        targetAngleDeg = Range.clip(degrees, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
        positionMode = true;
        manualPower = 0;
    }

    public double getTargetAngle() { return targetAngleDeg; }

    public boolean isAtTarget() {
        return positionMode && Math.abs(targetAngleDeg - getAngleDegrees()) <= ANGLE_TOLERANCE_DEG;
    }

    /** Manual motor power, still constrained by the configured angle limits. */
    public void setManualPower(double power) {
        requireInitialized();
        positionMode = false;
        manualPower = Range.clip(power, -MAX_POWER, MAX_POWER);
    }

    /** Stop commanding motion; hold with motor BRAKE (not active angle hold). */
    public void stop() {
        if (motor != null) motor.setPower(0);
        manualPower = 0;
        positionMode = false;
    }

    public boolean isPositionMode() { return positionMode; }

    @Override
    public void periodic() {
        if (motor == null) return;
        double angle = getAngleDegrees();
        double power;
        if (positionMode) {
            double error = targetAngleDeg - angle;
            power = Math.abs(error) <= ANGLE_TOLERANCE_DEG
                    ? 0.0 : Range.clip(KP * error, -MAX_POWER, MAX_POWER);
        } else {
            power = manualPower;
        }
        if (angle <= MIN_ANGLE_DEG && power < 0) power = 0;
        if (angle >= MAX_ANGLE_DEG && power > 0) power = 0;
        motor.setPower(power);
    }
}
