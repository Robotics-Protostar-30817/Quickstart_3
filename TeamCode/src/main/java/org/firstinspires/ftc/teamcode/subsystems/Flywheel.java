package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.subsystems.Subsystem;

/**
 * Single-motor flywheel subsystem for a goBILDA Yellow Jacket Planetary Gear Motor.
 *
 * The motor must be configured in the FTC Robot Configuration as a compatible
 * encoder-equipped motor, with the correct gear ratio / ticks-per-revolution.
 * Velocity values are MOTOR-ENCODER ticks per second, not flywheel RPM.
 * The belt drive has a confirmed 1:1 pulley ratio, so flywheel RPM equals
 * motor output-shaft RPM when the belt does not slip.
 *
 * This subsystem controls only the flywheel, not turret, hood or feeder.
 */
public final class Flywheel implements Subsystem {

    public static final Flywheel INSTANCE = new Flywheel();

    public static final double RATED_MOTOR_RPM = 6000.0; // nominal, not a commanded speed limit
    public static final double BELT_RATIO = 1.0; // flywheel RPM / motor output-shaft RPM

    private DcMotorEx motor;
    private double targetPower;
    private double targetVelocity;
    private double velocityTolerance = 100.0; // motor encoder ticks/second; tune experimentally
    private boolean velocityMode;

    //for auto aiming
    private static final double RPM_TOLERANCE = 150.0;
    private static final double READY_STABLE_SECONDS = 0.2;
    private double targetFlywheelRPM=0.0;
    private double readySinceSeconds=-1.0;
    private boolean readyToShoot = false;

    private final ElapsedTime timer = new ElapsedTime();
    private Flywheel() { }

    /** Call once from the OpMode's init(), with the configured Yellow Jacket motor. */
    public void init(DcMotorEx flywheelMotor) {
        if (flywheelMotor == null) {
            throw new IllegalArgumentException("Flywheel motor cannot be null");
        }
        motor = flywheelMotor;
        motor.setPower(0.0);
        // Verify that FORWARD produces the correct physical launch direction.
        //motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        targetPower = 0.0;
        targetVelocity = 0.0;
        velocityMode = false;
    }

    private void requireInitialized() {
        if (motor == null) {
            throw new IllegalStateException("Call Flywheel.init() first");
        }
    }

    /** Direct motor-power control for initial bench testing (0.0 to 1.0). */
    public void setPower(double power) {
        requireInitialized();
        if (!Double.isFinite(power)) {
            throw new IllegalArgumentException("Power must be finite");
        }
        targetPower = Math.max(0.0, Math.min(1.0, power));
        targetVelocity = 0.0;
        velocityMode = false;
        motor.setPower(targetPower);
    }

    /**
     * Built-in FTC closed-loop velocity control in MOTOR encoder ticks/second.
     * Use only after verifying encoder readings and determining a safe target.
     */
    public void setVelocity(double ticksPerSecond) {
        requireInitialized();
        if (!Double.isFinite(ticksPerSecond) || ticksPerSecond < 0.0) {
            throw new IllegalArgumentException("Velocity must be finite and nonnegative");
        }
        targetVelocity = ticksPerSecond;
        targetPower = 0.0;
        velocityMode = ticksPerSecond > 0.0;
        if (velocityMode) {
            motor.setVelocity(ticksPerSecond);
        } else {
            motor.setPower(0.0);
        }
    }

    public double getVelocity() {
        return motor == null ? 0.0 : motor.getVelocity();
    }

    /** Encoder counts per motor output-shaft revolution from FTC motor configuration. */
    public double getTicksPerRevolution() {
        requireInitialized();
        double ticks = motor.getMotorType().getTicksPerRev();
        if (!Double.isFinite(ticks) || ticks <= 0.0) {
            throw new IllegalStateException("Configure the correct Yellow Jacket motor type / encoder CPR");
        }
        return ticks;
    }

    /** Measured motor output-shaft speed; negative values indicate reverse rotation. */
    public double getMotorRPM() {
        return getVelocity() * 60.0 / getTicksPerRevolution();
    }

    /** Calculated flywheel speed; assumes a non-slipping 1:1 belt. */
    public double getFlywheelRPM() {
        return getMotorRPM() * BELT_RATIO;
    }

    /** Command motor output-shaft RPM using FTC encoder velocity control. */
    public void setMotorRPM(double rpm) {
        if (!Double.isFinite(rpm) || rpm < 0.0) {
            throw new IllegalArgumentException("RPM must be finite and nonnegative");
        }
        setVelocity(rpm * getTicksPerRevolution() / 60.0);
    }

    /** Command flywheel RPM using the belt ratio. */
    public void setFlywheelRPM(double rpm) {
        if (motor == null){
            return;
        }
        double requestedRPM = Math.max(0.0, Math.min(RATED_MOTOR_RPM, rpm));

        if (Math.abs(requestedRPM - targetFlywheelRPM) > 1.0){
            readyToShoot = false;
            readySinceSeconds = -1.0;
        }

        targetFlywheelRPM = requestedRPM;

        setMotorRPM(requestedRPM / BELT_RATIO);
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getTargetPower() {
        return targetPower;
    }

    public boolean isVelocityMode() {
        return velocityMode;
    }

    public void setVelocityTolerance(double ticksPerSecond) {
        if (!Double.isFinite(ticksPerSecond) || ticksPerSecond <= 0.0) {
            throw new IllegalArgumentException("Tolerance must be positive and finite");
        }
        velocityTolerance = ticksPerSecond;
    }

    /** Readiness applies only to velocity control, not direct power testing. */
    public boolean isAtSpeed() {
        return motor != null && velocityMode && targetVelocity > 0.0
                && Math.abs(getVelocity() - targetVelocity) <= velocityTolerance;
    }

    /** Removes drive power; FLOAT means the wheel may continue coasting. */
    public void stop() {
        targetPower = 0.0;
        targetVelocity = 0.0;
        velocityMode = false;
        targetFlywheelRPM = 0.0;
        readyToShoot = false;
        readySinceSeconds = -1.0;
        if (motor != null) {
            motor.setPower(0.0);
        }
    }

    public boolean isReadyToShoot(){
        return readyToShoot;
    }

    public double getTargetFlywheelRPM(){
        return targetFlywheelRPM;
    }

    /*
    * In the autoAiming process, the function should do the following:
    * 1. Read the actual flywheel RPM from the motor.
    * 2. Compare with the target RPM calculated by AutoAim.java.
    * 3. Apply velocity control to maintain the target speed.
    * 4. Determine whether the flywheel is ready to shoot.
     */
    @Override
    public void periodic() {
        // FTC motor controller handles RUN_USING_ENCODER velocity regulation.
        if (motor == null) {
            readyToShoot = false;
            readySinceSeconds = -1.0;
            return;
        }

        if (targetFlywheelRPM<=0.0){
            readyToShoot = false;
            readySinceSeconds = -1.0;
            return;
        }

        double actualFlywheelRPM = getFlywheelRPM();
        double error = Math.abs(targetFlywheelRPM - actualFlywheelRPM);

        boolean withinTolerance = error <= RPM_TOLERANCE;
        if (withinTolerance) {
            //start stability timer on first qualifying loop
            if (readySinceSeconds<0.0){
                readySinceSeconds = timer.seconds();
            }

            //Require stable speed for 200 ms.
            readyToShoot = timer.seconds() - readySinceSeconds >= READY_STABLE_SECONDS;
        }else{
            readySinceSeconds = -1.0;
            readyToShoot = false;
        }
    }
}

