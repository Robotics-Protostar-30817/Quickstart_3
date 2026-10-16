
package org.firstinspires.ftc.teamcode.pedro.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Locale;

/**
 * Checks path completion and retries navigation from the current pose.
 *
 * This class does NOT handle localization communication errors.
 * The caller must ensure that follower.pose() is reliable.
 */
public final class PathCompletionRecovery {

    public enum Result {
        RUNNING,
        RECOVERING,
        ARRIVED,
        FAILED
    }

    /** Builds a collision-safe replacement path. */
    public interface RecoveryPlanner {
        Path plan(Pose currentPose, Pose targetPose);
    }

    /** Supplies elapsed autonomous time in seconds. */
    public interface Clock {
        double seconds();
    }

    private static final String TAG = "PathCompletionRecovery";

    private final Follower follower;
    private final Clock clock;
    private final double positionTolerance;
    private final double headingTolerance;
    private final double idleGraceSeconds;
    private final int maxRetries;

    private Pose target;
    private RecoveryPlanner planner;
    private boolean verifyHeading;
    private boolean active;

    private int retries;
    private double idleSince = -1.0;
    private String taskName = "";
    private String failureReason = "";

    //handling timeout, forever running tasks.
    private final double noProgressTimeoutSeconds;
    private final double maxAttemptSeconds;
    private final double minProgressInches;
    private double attemptStartedAt =-1.0;
    private double lastProgressAt = -1.0;
    private double bestDistance = Double.MAX_VALUE;

    /**
     * Configures completion verification and retry behavior.
     *
     * @param follower Pedro follower
     * @param clock elapsed-time source
     * @param positionToleranceInches allowed XY error
     * @param headingToleranceDegrees allowed heading error
     * @param idleGraceSeconds delay before attempting recovery
     * @param maxRetries maximum retries for each navigation task
     * @param noProgressTimeoutSeconds timeout for no progress
     * @param maxAttemptSeconds max timeout for path duration
     * @param minProgressInches minimum distance to the target allowed for progress
     */
    public PathCompletionRecovery(
            Follower follower,
            Clock clock,
            double positionToleranceInches,
            double headingToleranceDegrees,
            double idleGraceSeconds,
            int maxRetries,
            double noProgressTimeoutSeconds,
            double maxAttemptSeconds,
            double minProgressInches) {

        if (follower == null || clock == null) {
            throw new IllegalArgumentException(
                    "Follower and clock are required");
        }

        if (positionToleranceInches < 0
                || headingToleranceDegrees < 0
                || idleGraceSeconds < 0
                || maxRetries < 0) {
            throw new IllegalArgumentException(
                    "Configuration must be nonnegative");
        }

        this.follower = follower;
        this.clock = clock;
        this.positionTolerance = positionToleranceInches;
        this.headingTolerance =
                Math.toRadians(headingToleranceDegrees);
        this.idleGraceSeconds = idleGraceSeconds;
        this.maxRetries = maxRetries;
        this.noProgressTimeoutSeconds = noProgressTimeoutSeconds;
        this.maxAttemptSeconds = maxAttemptSeconds;
        this.minProgressInches = minProgressInches;
    }

    /**
     * Starts progress monitoring for a new path attempt.
     * @param currentPose current robot pose
     */
    private void resetProgressTracking(Pose currentPose){
        double now = clock.seconds();
        attemptStartedAt = now;
        lastProgressAt = now;
        bestDistance = Math.hypot(currentPose.x()-target.x(),
                currentPose.y()-target.y());
    }

    /**
     * Begins a navigation task and remembers its target.
     *
     * @param name task name for logging
     * @param originalPath original Pedro path
     * @param targetPose expected destination
     * @param checkHeading whether final heading must match
     * @param recoveryPlanner safe replacement-path generator
     */
    public void begin(
            String name,
            Path originalPath,
            Pose targetPose,
            boolean checkHeading,
            RecoveryPlanner recoveryPlanner) {

        if (originalPath == null || targetPose == null
                || recoveryPlanner == null) {
            throw new IllegalArgumentException(
                    "Path, target, and planner are required");
        }

        cancel();

        taskName = name;
        target = targetPose;
        planner = recoveryPlanner;
        verifyHeading = checkHeading;

        retries = 0;
        idleSince = -1.0;
        failureReason = "";
        active = true;

        follower.follow(originalPath);
        resetProgressTracking(follower.pose());

        RobotLog.ii(TAG,
                "BEGIN %s target=(%.1f, %.1f, %.1f)",
                taskName,
                target.x(),
                target.y(),
                Math.toDegrees(target.heading()));
    }

    /**
     * Stops the current path attempt and starts a replacement path.
     * @param currentPose current robot pose
     * @param reason reason the current attempt was abandoned
     * @param remainingSeconds available time for navigation.
     * @return RECOVERING if restarted, otherwise FAILED
     */
    private Result recover(Pose currentPose, String reason, double remainingSeconds){

        if (retries >= maxRetries) {
            follower.stop();
            return fail("Retry limit reached: "+reason);
        }
        if (remainingSeconds <= 0) {
            follower.stop();
            return fail("No time remaining for recovery");
        }
        Path replacement;
        try{
            replacement = planner.plan(currentPose,target);
        }catch(RuntimeException e){
            follower.stop();
            return fail("Recovery planner failed: "+e.getMessage());
        }

        if (replacement == null) {
            follower.stop();
            return fail("No safe recovery path found");
        }

        RobotLog.ww(TAG,"RECOVER %s attempt=%d reason=%s from=(%.1f, %.1f, %.1f)",
                taskName,retries+1,reason,currentPose.x(),currentPose.y(),
                Math.toDegrees(currentPose.heading()));
        follower.stop();
        follower.follow(replacement);
        retries++;
        idleSince=-1;
        resetProgressTracking(currentPose);
        return Result.RECOVERING;
    }


    /**
     * Check how close Robot is to the target position
     */
    boolean closeEnough (Pose current, Pose target,
                         double positionTolerance,double headingTolerance){
        double distance = Math.hypot(current.x()-target.x(),
                current.y()-target.y());
        double headingError = Math.abs(angleDifference(current.heading(),target.heading()));
        if ((distance <=positionTolerance) &&
                (!verifyHeading || headingError <= headingTolerance))
            return true;
        else
            return false;
    }

    /**
     * Checks arrival and retries an incomplete path.
     * Monitors navigation and recovers from incomplete or stuck paths.
     *
     * Call once per loop after follower.update(), but only
     * when localization data is reliable.
     *
     * @param remainingNavigationSeconds remaining retry time budget
     * @return navigation result
     */
    public Result update(double remainingNavigationSeconds) {

        if (!active) {
            return Result.FAILED;
        }

        Pose current = follower.pose();

        if (current == null
                || !Double.isFinite(current.x())
                || !Double.isFinite(current.y())
                || !Double.isFinite(current.heading())) {
            return fail("Invalid current pose");
        }

        if (follower.isBusy()) {
            idleSince = -1.0;
           // if (closeEnough(current,target)){
           //     lastProgressAt = clock.seconds();
           //     return Result.RUNNING;
           // }

            String timeoutReason = checkPathTimeout(current);
            if (timeoutReason != null) {
                return recover(current,
                        timeoutReason,remainingNavigationSeconds);
            }
            return Result.RUNNING;
        }

        if (closeEnough(current, target,positionTolerance,headingTolerance)){

            active = false;
            RobotLog.ii(TAG,
                    "ARRIVED %s retries=%d",
                    taskName,
                    retries);

            return Result.ARRIVED;
        }

        double now = clock.seconds();

        if (idleSince < 0) {
            idleSince = now;
        }

        if (now - idleSince < idleGraceSeconds) {
            return Result.RUNNING;
        }

        return recover(current,"Follower idle before reaching target",
                remainingNavigationSeconds);
    }

    /**
     * Cancels task tracking without commanding motors.
     * The Auto must call follower.stop() if necessary.
     */
    public void cancel() {
        active = false;
        target = null;
        planner = null;
        idleSince = -1.0;
    }

    /** @return true if a navigation task is active */
    public boolean isActive() {
        return active;
    }

    /** @return number of retries started for this task */
    public int getRetries() {
        return retries;
    }

    /** @return last failure reason */
    public String getFailureReason() {
        return failureReason;
    }

    /**
     * Records a terminal navigation failure.
     *
     * @param reason failure description
     * @return FAILED
     */
    private Result fail(String reason) {
        active = false;
        failureReason = reason;
        follower.stop();
        RobotLog.ee(TAG,
                "FAILED %s retries=%d reason=%s",
                taskName, retries, reason);

        return Result.FAILED;
    }

    /**
     * Computes the shortest signed heading difference.
     *
     * @param a first heading in radians
     * @param b second heading in radians
     * @return difference in [-pi, pi]
     */
    private static double angleDifference(double a, double b) {
        return Math.atan2(
                Math.sin(a - b),
                Math.cos(a - b));
    }

    /**
     * Detects insufficient progress or excessive path duration.
     * Progress is measured as improvement in distance to the target.
     * This is suitable for routes that generally approach their targets.
     *
     * @param currentPose current robot pose
     * @return reason for recovery, or null if no timeout occurred.
     */
    private String checkPathTimeout(Pose currentPose){
        double now = clock.seconds();
        double distance = Math.hypot(currentPose.x()-target.x(),
                currentPose.y()-target.y());
        if (bestDistance - distance >= minProgressInches) {
            bestDistance = distance;
            lastProgressAt = now;
        }

        if (now - attemptStartedAt >= maxAttemptSeconds) {
            return String.format(Locale.US,"Path duration exceeds %.1f seconds",
                    maxAttemptSeconds);
        }

        if (now - lastProgressAt >= noProgressTimeoutSeconds) {
            return String.format(Locale.US,"No progress for %.1f seconds, distance = %.1f",
                    noProgressTimeoutSeconds, distance);
        }
        return null;
    }
}

