package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Stationary, vision-guided aiming. Call periodic() once per OpMode loop.
 * Initialize Vision, Turret, Hood and Flywheel first. Never actuates the feeder.
 * CELL coordinates must be validated as robot-centered inches; turret zero is forward.
 */
public final class AutoAim implements dev.nextftc.core.subsystems.Subsystem {
    public static final AutoAim INSTANCE = new AutoAim();

    // COPY VERIFIED SUCCESSFUL ROWS from ShotCalibration here, sorted by distance.
    // Each row: {horizontal distance in inches, hood servo position, flywheel RPM}.
    // At least two distinct distance rows required. No extrapolation.
    public static final double[][] SHOT_TABLE = {
             {36.0, 0.48, 2500.0},  // EXAMPLE ONLY: DO NOT UNCOMMENT UNTIL MEASURED
             {60.0, 0.50, 3000.0},
    };
    public static boolean CALIBRATION_VERIFIED = false;//after fill in the table with experimented values, set to true
    public static double TURRET_DIRECTION = 1.0; // set -1 if positive angle turns left
    public static double TURRET_FORWARD_OFFSET_DEG = 0.0;
    public static double MAX_TARGET_CHANGE_DEG = 5.0; // optional telemetry reference only

    //approximate hood settling time, measure and tune on the robot
    public static double HOOD_SETTLE_SECONDS = 0.5; // for hood readiness check
    public static double HOOD_CHANGE_THRESHOLD = 0.002; // for hood readiness check
    private final Vision vision = Vision.INSTANCE;
    private final Turret turret = Turret.INSTANCE;
    private final Hood hood = Hood.INSTANCE;
    private final Flywheel flywheel = Flywheel.INSTANCE;
    private Vision.Cell.Color alliance = Vision.Cell.Color.RED;
    private boolean enabled;
    private boolean ready;
    private String status = "Disabled";
    private double distanceInches = Double.NaN;
    private double targetTurretDeg = Double.NaN;
    private double targetHoodPosition = Double.NaN;
    private double targetRpm = Double.NaN;

    private final ElapsedTime hoodTimer = new ElapsedTime();
    private double lastHoodCommand = Double.NaN;
    private boolean hoodSettlingStarted;
    private boolean hoodReady;

    private AutoAim() { }

    public void init(Vision.Cell.Color color) {
        if (color != Vision.Cell.Color.RED && color != Vision.Cell.Color.BLUE)
            throw new IllegalArgumentException("Alliance must be RED or BLUE");
        alliance = color;
        enabled = false;
        ready = false;
        status = "Disabled";
        lastHoodCommand = Double.NaN;
        hoodSettlingStarted = false;
        hoodReady = false;
    }

    public void enable() { enabled = true; }
    public void disable() {
        enabled = false;
        safeStop("Disabled");
    }
    public boolean isEnabled() { return enabled; }
    /** Readiness is approximate: positional servos have no hood position feedback. */
    public boolean isReadyToShoot() { return ready; }
    public String getStatus() { return status; }
    public double getDistanceInches() { return distanceInches; }
    public double getTargetTurretDegrees() { return targetTurretDeg; }
    public double getTargetHoodPosition() { return targetHoodPosition; }
    public double getTargetFlywheelRPM() { return targetRpm; }

    private void safeStop(String reason) {
        ready = false;
        hoodReady = false;
        lastHoodCommand = Double.NaN;
        status = reason;
        distanceInches = targetTurretDeg = targetHoodPosition = targetRpm = Double.NaN;
        turret.stop();
        flywheel.stop();
        // Hood servo remains at its last commanded position.
    }

    @Override public void periodic() {
        if (!enabled) return;
        Vision.Cell cell = vision.getBestCell(alliance);
        if (cell == null || !cell.isScorable()) { safeStop("No scorable CELL"); return; }
        double x = cell.getX(), y = cell.getY(), z = cell.getZ();
        if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(z)) {
            safeStop("Invalid CELL XYZ"); return;
        }
        double distance = Math.hypot(x, y);
        if (distance <= 1.0) { safeStop("Invalid distance"); return; }
        double direction = TURRET_DIRECTION * Math.toDegrees(Math.atan2(x, y))
                + TURRET_FORWARD_OFFSET_DEG;
        if (!Double.isFinite(direction) || direction < Turret.MIN_ANGLE_DEG
                || direction > Turret.MAX_ANGLE_DEG) {
            safeStop("CELL outside turret limits"); return;
        }
        if (!CALIBRATION_VERIFIED || !validTable()) {
            safeStop("Shot table missing/unverified"); return;
        }
        if (distance < SHOT_TABLE[0][0] || distance > SHOT_TABLE[SHOT_TABLE.length - 1][0]) {
            safeStop("Distance outside calibrated range"); return;
        }
        double hoodPos = Double.NaN, rpm = Double.NaN;
        for (int i = 0; i < SHOT_TABLE.length - 1; i++) {
            double[] a = SHOT_TABLE[i], b = SHOT_TABLE[i + 1];
            if (distance >= a[0] && distance <= b[0]) {
                double fraction = (distance - a[0]) / (b[0] - a[0]);
                hoodPos = a[1] + fraction * (b[1] - a[1]);
                rpm = a[2] + fraction * (b[2] - a[2]);
                break;
            }
        }
        if (!Double.isFinite(hoodPos) || !Double.isFinite(rpm)
                || hoodPos < Hood.MIN_POSITION || hoodPos > Hood.MAX_POSITION
                || rpm <= 0 || rpm > Flywheel.RATED_MOTOR_RPM) {
            safeStop("Shot command outside limits"); return;
        }
        distanceInches = distance;
        targetTurretDeg = direction;
        targetHoodPosition = hoodPos;
        targetRpm = rpm;

        turret.setTargetAngle(direction);
        if (!hoodSettlingStarted || Math.abs(hoodPos - lastHoodCommand) > HOOD_CHANGE_THRESHOLD) {
            hoodTimer.reset();
            hoodSettlingStarted = true;
            lastHoodCommand = hoodPos;
        }
        if (hoodSettlingStarted && hoodTimer.seconds() >= HOOD_SETTLE_SECONDS) {
            hoodReady = true;
        }

        hood.setPosition(hoodPos);
        flywheel.setFlywheelRPM(rpm);
        turret.periodic();
        hood.periodic();
        flywheel.periodic();
        ready = turret.isAtTarget() && flywheel.isReadyToShoot() && hoodReady;
        if (ready) status = "Ready (hood time-estimated, not measured";
        else if (!hoodReady) status = "Waiting for hood to settle";
        else status = "Aiming/waiting for flywheel";
    }

    private boolean validTable() {
        if (SHOT_TABLE.length < 2) return false;
        double previousDistance = Double.NEGATIVE_INFINITY;
        for (double[] row : SHOT_TABLE) {
            if (row == null || row.length != 3 || !Double.isFinite(row[0])
                    || !Double.isFinite(row[1]) || !Double.isFinite(row[2])
                    || row[0] <= previousDistance || row[0] <= 0
                    || row[1] < Hood.MIN_POSITION || row[1] > Hood.MAX_POSITION
                    || row[2] <= 0 || row[2] > Flywheel.RATED_MOTOR_RPM) return false;
            previousDistance = row[0];
        }
        return true;
    }
}

