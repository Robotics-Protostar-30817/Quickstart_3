package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
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

    //Estimated servo settling: calibrate on the physical robot
    public static double SETTLE_SECONDS = 0.25;
    public static double CHANGE_THRESHOLD = 0.002;

    /** Replace with measured {servo position, physical launch angle in degrees} pairs.
     *  Positions must strictly increase; angles must be strictly monotonic.
     *  Example rows below are placeholders ONLY. */
    public static final double[][] ANGLE_CALIBRATION = {
            {0.45, Double.NaN},
            {0.55, Double.NaN}
    };
    public static boolean ANGLE_CALIBRATION_VERIFIED = false;

    private Servo servo;
    private double commandedPosition = 0.5;
    private boolean hasCommand = false;
    //for readiness check up
    private final ElapsedTime timer = new ElapsedTime();
    private double lastSettlingPosition = Double.NaN;
    private boolean settlingStarted = false;
    private double targetLaunchAngleDegrees = Double.NaN;

    private Hood() { }

    public void init(Servo hoodServo) {
        if (hoodServo == null) throw new IllegalArgumentException("hoodServo is null");
        servo = hoodServo;
        // Do not move the mechanism automatically at initialization.
        commandedPosition = Range.clip(servo.getPosition(), 0.0, 1.0);
        hasCommand = false;
        targetLaunchAngleDegrees = Double.NaN;
        invalidateReadiness();
    }
    //Invalidate readiness after loss of target or disabling auto aim.
    public void invalidateReadiness() {
        //timer.reset();
        lastSettlingPosition = Double.NaN;
        settlingStarted = false;
    }

    //Time-based estimate only:
    //a standard positional servo has no position error.
    public boolean isReady(){
        return servo !=null && hasCommand && settlingStarted && timer.seconds() >= SETTLE_SECONDS;
    }

    private void requireInitialized() {
        if (servo == null) throw new IllegalStateException("Hood.init() not called");
    }

    public void setPosition(double position) {
        requireInitialized();
        if (MIN_POSITION > MAX_POSITION) throw new IllegalStateException("Invalid hood position limits");
        double nextPosition = Range.clip(position, MIN_POSITION, MAX_POSITION);
        if (!settlingStarted || Math.abs(nextPosition - lastSettlingPosition) > CHANGE_THRESHOLD) {
            timer.reset();
            settlingStarted = true;
            lastSettlingPosition = nextPosition;
        }
        commandedPosition = nextPosition;
        targetLaunchAngleDegrees = Double.NaN;
        servo.setPosition(commandedPosition);
        hasCommand = true;
    }

    /** Convert physical launch angle to a servo command using linear interpolation.
     *  Reject uncalibrated angles and extrapolation; never silently clip an angle. */
    public void setLaunchAngle(double angleDegrees) {
        double position = servoPositionForAngle(angleDegrees);
        setPosition(position);
        targetLaunchAngleDegrees = angleDegrees;
    }

    public double servoPositionForAngle(double angleDegrees) {
        if (!hasValidAngleCalibration())
            throw new IllegalStateException("Hood angle calibration missing/unverified");
        if (!Double.isFinite(angleDegrees))
            throw new IllegalArgumentException("Invalid hood launch angle");
        double first = ANGLE_CALIBRATION[0][1];
        double last = ANGLE_CALIBRATION[ANGLE_CALIBRATION.length - 1][1];
        if (angleDegrees < Math.min(first, last) || angleDegrees > Math.max(first, last))
            throw new IllegalArgumentException("Hood angle outside calibrated range");
        for (int i = 0; i < ANGLE_CALIBRATION.length - 1; i++) {
            double[] a = ANGLE_CALIBRATION[i], b = ANGLE_CALIBRATION[i + 1];
            if (angleDegrees >= Math.min(a[1], b[1]) && angleDegrees <= Math.max(a[1], b[1])) {
                double fraction = (angleDegrees - a[1]) / (b[1] - a[1]);
                return a[0] + fraction * (b[0] - a[0]);
            }
        }
        throw new IllegalStateException("Cannot interpolate hood launch angle");
    }

    public boolean hasValidAngleCalibration() {
        if (!ANGLE_CALIBRATION_VERIFIED || ANGLE_CALIBRATION == null
                || ANGLE_CALIBRATION.length < 2) return false;
        int direction = 0;
        for (int i = 0; i < ANGLE_CALIBRATION.length; i++) {
            double[] row = ANGLE_CALIBRATION[i];
            if (row == null || row.length != 2 || !Double.isFinite(row[0])
                    || !Double.isFinite(row[1]) || row[0] < MIN_POSITION
                    || row[0] > MAX_POSITION) return false;
            if (i > 0) {
                double[] prev = ANGLE_CALIBRATION[i - 1];
                if (row[0] <= prev[0] || row[1] == prev[1]) return false;
                int stepDirection = row[1] > prev[1] ? 1 : -1;
                if (direction != 0 && direction != stepDirection) return false;
                direction = stepDirection;
            }
        }
        return true;
    }
    public void adjustPosition(double delta) {
        setPosition(commandedPosition + delta);
    }

    /** Last requested position; not independent position feedback. */
    public double getPosition() {
        requireInitialized();
        return commandedPosition;
    }

    /** Last requested physical launch angle; not sensor feedback. */
    public double getTargetLaunchAngleDegrees() { return targetLaunchAngleDegrees; }
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
