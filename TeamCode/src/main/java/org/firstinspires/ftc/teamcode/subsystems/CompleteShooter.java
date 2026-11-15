package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;
import dev.nextftc.core.subsystems.Subsystem;

/** Coordinates intake, vision-guided aiming, blocker and upper feed motor.
 *  Continuous feed does NOT detect when storage is empty.
 *  Call periodic() exactly once per OpMode loop; do not also call AutoAim.periodic().
 */
public final class CompleteShooter implements Subsystem {
    public static final CompleteShooter INSTANCE = new CompleteShooter();

    // All values are starting points, NOT verified hardware calibrations.
    public static double INTAKE_POWER = 0.25;
    public static double FEED_POWER = 0.25;
    public static double IDLE_FLYWHEEL_RPM = 800.0;
    public static double BLOCKER_OPEN_SECONDS = 0.20;
    // Driver stops continuous shooting with cancelShot().
    // Optional watchdogs: 0 disables the timeout (default).
    // A positive value limits feed or aim-wait time for testing/safety.
    public static double MAX_CONTINUOUS_FEED_SECONDS = 0.0;
    public static double REQUEST_TIMEOUT_SECONDS = 0.0;

    private enum Stage { IDLE, WAITING, OPENING, FEEDING }
    private Stage stage = Stage.IDLE;
    private final AutoAim aim = AutoAim.INSTANCE;
    private final Flywheel flywheel = Flywheel.INSTANCE;
    private final Transfer transfer = Transfer.INSTANCE;
    private final Blocker blocker = Blocker.INSTANCE;
    private final IntakeRamp ramp = IntakeRamp.INSTANCE;
    private final ElapsedTime stageTimer = new ElapsedTime();
    private final ElapsedTime requestTimer = new ElapsedTime();
    private boolean initialized, collecting;
    private String status = "Uninitialized";

    private CompleteShooter() {}

    /** Call after Vision, Turret, Hood, Flywheel, Transfer, Blocker and IntakeRamp init(). */
    public void init(Vision.Cell.Color alliance) {
        transfer.stop();
        ramp.stop();
        blocker.close();
        flywheel.stop();
        aim.init(alliance);
        collecting = false;
        stage = Stage.IDLE;
        initialized = true;
        status = "Idle";
    }

    private void requireInit() {
        if (!initialized) throw new IllegalStateException("CompleteShooter.init() required");
    }

    /** Lower transfer motor + ramp only. Upper motor and blocker remain independent. */
    public void startCollecting() {
        requireInit();
        if (!Double.isFinite(INTAKE_POWER) || INTAKE_POWER <= 0 || INTAKE_POWER > 1)
            throw new IllegalStateException("Invalid intake power");
        transfer.setIntakePower(INTAKE_POWER);
        ramp.intake();
        collecting = true;
    }

    public void stopCollecting() {
        if (!initialized) return;
        transfer.stopIntake();
        ramp.stop();
        collecting = false;
    }

    public boolean isCollecting() { return collecting; }
    public void enableAim() { requireInit(); aim.enable(); }
    public void disableAim() { requireInit(); cancelShot(); aim.disable(); }

    /** One-button command: enable AutoAim and start continuous shooting once ready.
     * Continues until driver calls cancelShot(), or a safety fault.
     * Does not interrupt floor collection. */
    public void requestShot() {
        requireInit();
        if (stage != Stage.IDLE) return;
        aim.enable(); // One button starts aiming and queues continuous shooting.
        stage = Stage.WAITING;
        requestTimer.reset();
        status = "Continuous shooting requested; waiting for aim";
    }

    public void cancelShot() {
        if (!initialized) return;
        transfer.stopFeed();
        if (initialized) blocker.close();
        stage = Stage.IDLE;
        status = "Shooting stopped by driver/cancel";
    }

    public boolean isFiring() { return stage == Stage.FEEDING; }
    public boolean isShootingActive() { return stage != Stage.IDLE; }
    /** True whenever AutoAim has a valid ready-to-fire solution. */
    public boolean isAimReady() { return initialized && aim.isReadyToShoot(); }
    /** No storage sensor is present; this method intentionally does not infer emptiness. */
    public boolean canDetectEmptyStorage() { return false; }
    public boolean isShotPending() { return stage == Stage.WAITING || stage == Stage.OPENING; }
    public boolean isReadyToShoot() {
        return isAimReady();
    }
    public String getStage() { return stage.name(); }
    public String getStatus() { return status; }

    private boolean validFeedSettings() {
        return Double.isFinite(FEED_POWER) && FEED_POWER > 0 && FEED_POWER <= 1
                && Double.isFinite(BLOCKER_OPEN_SECONDS) && BLOCKER_OPEN_SECONDS >= 0
                && Double.isFinite(MAX_CONTINUOUS_FEED_SECONDS) && MAX_CONTINUOUS_FEED_SECONDS >= 0
                && Double.isFinite(REQUEST_TIMEOUT_SECONDS) && REQUEST_TIMEOUT_SECONDS >= 0;
    }

    @Override public void periodic() {
        if (!initialized) return;
        // AutoAim alone updates turret, hood, and flywheel when enabled.
        aim.periodic();
        // Standby is commanded only when AutoAim is disabled, so it never
        // overrides an active aiming command or AutoAim safety stop.
        if (!aim.isEnabled()) {
            if (Double.isFinite(IDLE_FLYWHEEL_RPM) && IDLE_FLYWHEEL_RPM > 0
                    && IDLE_FLYWHEEL_RPM <= Flywheel.RATED_MOTOR_RPM) {
                flywheel.setFlywheelRPM(IDLE_FLYWHEEL_RPM);
                flywheel.periodic();
            } else {
                flywheel.stop();
            }
        }

        switch (stage) {
            case IDLE:
                transfer.stopFeed();
                blocker.close();
                status = aim.isEnabled() ? aim.getStatus() : "Idle (flywheel standby)";
                break;
            case WAITING:
                transfer.stopFeed();
                blocker.close();
                if (!aim.isEnabled()) {
                    cancelShot(); status = "Aim disabled";
                } else if (REQUEST_TIMEOUT_SECONDS > 0
                        && requestTimer.seconds() >= REQUEST_TIMEOUT_SECONDS) {
                    cancelShot(); status = "Shot request timed out";
                } else if (aim.isReadyToShoot()) {
                    if (!validFeedSettings()) {
                        cancelShot(); status = "Invalid feed calibration";
                    } else {
                        blocker.open();
                        stageTimer.reset();
                        stage = Stage.OPENING;
                        status = "Opening blocker";
                    }
                } else {
                    status = "Waiting: " + aim.getStatus();
                }
                break;
            case OPENING:
                if (!aim.isEnabled() || !aim.isReadyToShoot()) {
                    cancelShot(); status = "Aim not ready; blocker closed";
                } else if (stageTimer.seconds() >= BLOCKER_OPEN_SECONDS) {
                    transfer.setFeedPower(FEED_POWER);
                    stageTimer.reset();
                    stage = Stage.FEEDING;
                    status = "Feeding toward flywheel";
                }
                break;
            case FEEDING:
                // Keep feeding multiple POLLEN during one request. A brief flywheel
                // RPM drop caused by a launch does not terminate the cycle.
                // Stop on disabled aim, lost shot solution, or safety timeout.
                if (!aim.isEnabled()) {
                    cancelShot();
                    status = "Target lost; continuous feed stopped";
                } else if (MAX_CONTINUOUS_FEED_SECONDS > 0
                        && stageTimer.seconds() >= MAX_CONTINUOUS_FEED_SECONDS) {
                    cancelShot();
                    status = "Feed safety timeout; empty storage NOT confirmed";
                } else {
                    status = "Continuous feeding; driver stops when all POLLEN fired";
                }
                break;
        }
    }

    /** Emergency stop: no standby flywheel after stop(). */
    public void stop() {
        if (!initialized) return;
        cancelShot();
        stopCollecting();
        aim.disable();
        flywheel.stop();
        initialized = false;
        status = "Stopped";
    }
}
