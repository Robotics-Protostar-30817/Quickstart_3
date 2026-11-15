package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import dev.nextftc.core.subsystems.Subsystem;

/** Two independent paths: lower floor intake and upper feed to flywheel. */
public final class Transfer implements Subsystem {
    public static final Transfer INSTANCE = new Transfer();
    public static DcMotorSimple.Direction LOWER_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction UPPER_DIRECTION = DcMotorSimple.Direction.FORWARD;
    private DcMotorEx lower, upper;
    private double lowerPower, upperPower;
    private Transfer() {}

    /** motor1 = floor intake; motor2 = flywheel feed. */
    public void init(DcMotorEx motor1, DcMotorEx motor2) {
        if (motor1 == null || motor2 == null || motor1 == motor2)
            throw new IllegalArgumentException("Two distinct transfer motors required");
        lower = motor1; upper = motor2;
        lower.setDirection(LOWER_DIRECTION);
        upper.setDirection(UPPER_DIRECTION);
        for (DcMotorEx motor : new DcMotorEx[]{lower, upper}) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        stop();
    }
    private void checkInit() {
        if (lower == null || upper == null) throw new IllegalStateException("Transfer.init() required");
    }
    private static double power(double value) {
        if (!Double.isFinite(value)) throw new IllegalArgumentException("Non-finite motor power");
        return Math.max(-1, Math.min(1, value));
    }
    public void setIntakePower(double value) { checkInit(); lowerPower = power(value); lower.setPower(lowerPower); }
    public void setFeedPower(double value) { checkInit(); upperPower = power(value); upper.setPower(upperPower); }
    public void stopIntake() { lowerPower = 0; if (lower != null) lower.setPower(0); }
    public void stopFeed() { upperPower = 0; if (upper != null) upper.setPower(0); }
    public void stop() { stopIntake(); stopFeed(); }
    public double getIntakePower() { return lowerPower; }
    public double getFeedPower() { return upperPower; }
    public double getVelocity1() { return lower == null ? 0 : lower.getVelocity(); }
    public double getVelocity2() { return upper == null ? 0 : upper.getVelocity(); }
    /** Legacy API for older tests. Prefer independent commands above. */
    @Deprecated public void setPower(double value) { setIntakePower(value); setFeedPower(value); }
    @Deprecated public double getTargetPower() { return lowerPower; }
    @Deprecated public String getDirection() { return LOWER_DIRECTION.toString(); }
    @Deprecated public void toggleDirection() {
        checkInit(); stop();
        LOWER_DIRECTION = LOWER_DIRECTION == DcMotorSimple.Direction.FORWARD ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
        UPPER_DIRECTION = UPPER_DIRECTION == DcMotorSimple.Direction.FORWARD ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
        lower.setDirection(LOWER_DIRECTION); upper.setDirection(UPPER_DIRECTION);
    }
    @Override public void periodic() {}
}
