package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Manages time during the 30-second FTC autonomous period.
 *
 * <p>This class is responsible for determining when the robot must stop
 * performing normal autonomous tasks and begin its PARK operation. It also
 * determines whether enough time remains to safely begin another task.</p>
 *
 * <p>Each robot should create its own {@code AutoTimeManager}, because
 * Robot 1 and Robot 2 can have different worst-case return-to-PARK times.</p>
 *
 * <p>The PARK deadline is calculated as:</p>
 *
 * <pre>
 * PARK deadline =
 *     autonomous duration
 *     - worst-case PARK return time
 *     - PARK safety margin
 * </pre>
 *
 * <p>The class also maintains a separate timer for measuring how long
 * the robot spends in each autonomous state.</p>
 */
public class AutoTimeManager {

    /**
     * Length of the FTC autonomous period, in seconds.
     */
    public static final double AUTO_DURATION = 30.0;

    /**
     * Timer measuring total elapsed autonomous time.
     */
    private final ElapsedTime autoTimer = new ElapsedTime();

    /**
     * Timer measuring time spent in the current autonomous state.
     */
    private final ElapsedTime stateTimer = new ElapsedTime();

    /**
     * Maximum measured time required for this robot to return to
     * its designated PARK position, in seconds.
     */
    private final double worstCaseParkTime;

    /**
     * Additional time reserved beyond the measured worst-case PARK
     * time, in seconds.
     */
    private final double parkSafetyMargin;

    /**
     * Name of the current autonomous state.
     */
    private String currentState = "NOT_STARTED";

    /**
     * Creates an autonomous time manager for one robot.
     *
     * <p>The supplied PARK time should be determined experimentally by
     * testing the robot from the worst realistic autonomous positions.</p>
     *
     * @param worstCaseParkTime maximum measured return-to-PARK time,
     *                          in seconds
     * @param parkSafetyMargin additional safety time reserved for PARK,
     *                         in seconds
     * @throws IllegalArgumentException if either time is negative, or if
     *                                  their sum is greater than or equal
     *                                  to the autonomous period
     */
    public AutoTimeManager(
            double worstCaseParkTime,
            double parkSafetyMargin) {

        if (worstCaseParkTime < 0) {
            throw new IllegalArgumentException(
                    "worstCaseParkTime cannot be negative");
        }

        if (parkSafetyMargin < 0) {
            throw new IllegalArgumentException(
                    "parkSafetyMargin cannot be negative");
        }

        if (worstCaseParkTime + parkSafetyMargin
                >= AUTO_DURATION) {

            throw new IllegalArgumentException(
                    "PARK time plus safety margin must be less than "
                            + AUTO_DURATION + " seconds");
        }

        this.worstCaseParkTime = worstCaseParkTime;
        this.parkSafetyMargin = parkSafetyMargin;
    }

    /**
     * Starts the autonomous and state timers.
     *
     * <p>This method should normally be called from the autonomous
     * OpMode's {@code start()} method.</p>
     */
    public void start() {
        autoTimer.reset();
        stateTimer.reset();
        currentState = "START";
    }

    /**
     * Records the beginning of a new autonomous state.
     *
     * <p>The state timer is reset whenever this method is called.
     * The main autonomous timer is not affected.</p>
     *
     * @param stateName name of the newly entered autonomous state
     */
    public void startState(String stateName) {
        currentState = stateName;
        stateTimer.reset();
    }

    /**
     * Returns the total elapsed autonomous time.
     *
     * @return elapsed autonomous time, in seconds
     */
    public double getElapsedTime() {
        return autoTimer.seconds();
    }

    /**
     * Returns the amount of autonomous time remaining.
     *
     * <p>The returned value will never be less than zero.</p>
     *
     * @return remaining autonomous time, in seconds
     */
    public double getRemainingTime() {
        return Math.max(
                0.0,
                AUTO_DURATION - getElapsedTime());
    }

    /**
     * Returns the time spent in the current autonomous state.
     *
     * @return current state duration, in seconds
     */
    public double getStateTime() {
        return stateTimer.seconds();
    }

    /**
     * Returns the name of the current autonomous state.
     *
     * @return current state name
     */
    public String getCurrentState() {
        return currentState;
    }

    /**
     * Calculates the latest autonomous time at which PARK should begin.
     *
     * @return PARK deadline measured from the beginning of autonomous,
     *         in seconds
     */
    public double getParkDeadline() {
        return AUTO_DURATION
                - worstCaseParkTime
                - parkSafetyMargin;
    }

    /**
     * Determines whether the robot should begin PARK immediately.
     *
     * @return {@code true} if the PARK deadline has been reached;
     *         {@code false} otherwise
     */
    public boolean shouldPark() {
        return getElapsedTime() >= getParkDeadline();
    }

    /**
     * Determines whether there is enough remaining autonomous time
     * to safely begin a proposed task.
     *
     * <p>The calculation reserves enough time after the proposed task
     * for both the worst-case PARK return and the PARK safety margin.</p>
     *
     * @param estimatedTaskTime estimated duration of the proposed task,
     *                          in seconds
     * @return {@code true} if the task can safely begin;
     *         {@code false} otherwise
     */
    public boolean canStartTask(double estimatedTaskTime) {

        if (estimatedTaskTime < 0) {
            return false;
        }

        double requiredTime =
                estimatedTaskTime
                        + worstCaseParkTime
                        + parkSafetyMargin;

        return getRemainingTime() >= requiredTime;
    }

    /**
     * Returns the configured worst-case PARK return time.
     *
     * @return worst-case PARK return time, in seconds
     */
    public double getWorstCaseParkTime() {
        return worstCaseParkTime;
    }

    /**
     * Returns the configured PARK safety margin.
     *
     * @return PARK safety margin, in seconds
     */
    public double getParkSafetyMargin() {
        return parkSafetyMargin;
    }
}