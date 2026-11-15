package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

/** Coordinates AutoAim and Transfer. Does not count actual POLLEN. */
public final class CompleteShooter implements dev.nextftc.core.subsystems.Subsystem {
    public static final CompleteShooter INSTANCE = new CompleteShooter();
    /** Calibrate on the real mechanism; one interval does not guarantee one POLLEN. */
    public static double TRANSFER_POWER = 0.45;
    public static double TRANSFER_SECONDS = 0.25;
    public static double MAX_TRANSFER_SECONDS = 0.75;
    /** Cancel a queued request if readiness never arrives. */
    public static double SHOT_REQUEST_TIMEOUT_SECONDS = 3.0;
    private final AutoAim aim = AutoAim.INSTANCE;
    private final Transfer transfer = Transfer.INSTANCE;
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime requestTimer = new ElapsedTime();
    private boolean firing, requestPending;
    private String status = "Uninitialized";

    private CompleteShooter() { }

    /** Call after initializing Vision, Turret, Hood, Flywheel and Transfer. */
    public void init(Vision.Cell.Color alliance) {
        transfer.stop();
        aim.init(alliance);
        firing = false;
        requestPending = false;
        status = "Idle";
    }

    public void enableAim() { aim.enable(); }
    public void disableAim() { cancel(); aim.disable(); }
    /** Queues one timed feed. It will start only while AutoAim is ready. */
    public void requestShot() {
        if (!firing && aim.isEnabled()) {
            requestPending = true;
            requestTimer.reset();
            status = "Shot queued";
        }
    }
    public void cancel() {
        requestPending = false;
        firing = false;
        transfer.stop();
        status = "Cancelled";
    }
    public boolean isFiring() { return firing; }
    public boolean isShotPending() { return requestPending; }
    public boolean isReadyToShoot() { return !firing && aim.isReadyToShoot(); }
    public String getStatus() { return status; }

    @Override public void periodic() {
        // AutoAim is the only owner of turret/hood/flywheel periodic updates.
        aim.periodic();
        if (firing) {
            // Stop feeding if aiming becomes invalid, or when the timed pulse ends.
            if (!aim.isEnabled() || !aim.isReadyToShoot()
                    || timer.seconds() >= TRANSFER_SECONDS
                    || timer.seconds() >= MAX_TRANSFER_SECONDS) {
                transfer.stop();
                firing = false;
                status = "Transfer ended (shot not sensor-confirmed)";
            }
            return;
        }
        transfer.stop();
        if (requestPending) {
            if (!aim.isEnabled()) { cancel(); return; }
            if (requestTimer.seconds() >= SHOT_REQUEST_TIMEOUT_SECONDS) {
                requestPending = false;
                status = "Shot request timed out";
                return;
            }
            if (!aim.isReadyToShoot()) { status = "Waiting: " + aim.getStatus(); return; }
            if (!Double.isFinite(TRANSFER_POWER) || TRANSFER_POWER <= 0 || TRANSFER_POWER > 1
                    || !Double.isFinite(TRANSFER_SECONDS) || TRANSFER_SECONDS <= 0
                    || TRANSFER_SECONDS > MAX_TRANSFER_SECONDS) {
                cancel(); status = "Invalid transfer calibration"; return;
            }
            requestPending = false;
            firing = true;
            timer.reset();
            transfer.setPower(TRANSFER_POWER);
            status = "Transferring POLLEN";
        } else {
            status = aim.isEnabled() ? aim.getStatus() : "Idle";
        }
    }
    public void stop() { cancel(); aim.disable(); }
}

