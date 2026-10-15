package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.auto.PathCompletionRecovery;
import org.firstinspires.ftc.teamcode.pedro.auto.RedScoringPaths;

/**
 * Red Scoring autonomous, navigation-only test.
 *
 * <p>Sequence: approach Red Scoring, wait for scorable status, simulate first
 * shot, travel to collection, simulate collection, return to Red Scoring,
 * simulate second shot, park. At 27 seconds initiate emergency parking.
 *
 * <p>Vision and mechanism integrations are deliberately disabled. The first
 * scorable event is simulated after two seconds in the WAIT currentState.
 */
@Autonomous(name = "Red Scoring Auto - State Machine", group = "Autonomous")
public final class RedScoringAuto extends OpMode {
    private enum AutoState {
        READY, INITIAL_APPROACH_Path, 
        WAIT_FOR_SCORING_SCORABLE, 
        SHOOT_FIRST,
        DRIVE_TO_FLOWER_Path, 
        COLLECT_FLOWER, 
        FLOWER_TO_SCORING_Path,
        SHOOT_SECOND, 
        DRIVE_TO_PARK_Path, 
        EMERGENCY_PARK_Path, 
        FINISHED,
        FAULT
    }

    private static final boolean NAVIGATION_ONLY = true;
    private static final double AUTO_SECONDS = 30.0;
    private static final double PARK_RESERVE_SECONDS = 3.0;
    private static final double TEST_SHOOT = 1.0;
    private static final double TEST_COLLECTION = 2.0;
    private static final double SIMULATED_SCORABLE_WAIT_SECONDS = 2.0;
    private static final double LOG_INTERVAL_SECONDS = 0.10;

    //for task completion, recovery and idle checking
    private static final double POSITION_TOLERANCE_INCHES = 3.0;
    private static final double HEADING_TOLERANCE_DEGREES = 15.0;
    private static final double IDLE_GRACE_SECONDS=0.25;
    private static final int MAX_RETRIES=3;
    private static final double NO_PROGRESS_TIMEOUT_SECONDS=1.0;
    private static final double MAX_ATTEMPTS_SECONDS=5.0;
    private static final double MIN_PROGRESS_INCHES=1;


    private static final double MAX_POSITION_JUMP_INCHES = 12.0;
    private static final double MAX_HEADING_JUMP_DEGREES = 100.0;

    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime logTimer = new ElapsedTime();
    private Follower follower;
    private AutoState currentState;
    private AutoState previousState;
    private RedScoringPaths paths;
    private String lastTransition = "Not Started";
    private String transitionReason="";
    private boolean emergyParkStarted;
    private boolean parkingPathStarted;
    //private AutoState currentState = AutoState.READY;
    private Pose previousPose;
    private Pose activeTarget;
    private boolean started;
    private boolean finished;
    private double pathNotBusySince = -1.0;
    private String faultReason = "";

    private PathCompletionRecovery navigation;
    private void logRobotStatus(String event){
        Pose p = follower.pose();
        RobotLog.ii("RedScoringAuto",
                "t=%.2f event=%s state=%s x=%.2f y=%.2f heading=%.1f busy=%b",
                autoTimer.seconds(),event,currentState,
                p.x(),p.y(),Math.toDegrees(p.heading()),follower.isBusy());
    }

    /** Initializes Pedro localization and the navigation-only test. */
    @Override
    public void init() {
        if (!NAVIGATION_ONLY) {
            throw new IllegalStateException("Mechanisms and Vision are not implemented");
        }
        paths = new RedScoringPaths();
        follower = Constants.create(hardwareMap);
        follower.setPose(paths.startPose);
        navigation = new PathCompletionRecovery(follower, ()->autoTimer.seconds(),
                POSITION_TOLERANCE_INCHES,HEADING_TOLERANCE_DEGREES,IDLE_GRACE_SECONDS,
                MAX_RETRIES,NO_PROGRESS_TIMEOUT_SECONDS, MAX_ATTEMPTS_SECONDS,
                MIN_PROGRESS_INCHES);
        previousPose = follower.pose();
        currentState = AutoState.READY;
        started = false;
        parkingPathStarted = false;
        activeTarget = null;
        pathNotBusySince = -1.0;
        faultReason = "";
        telemetry.addLine("Red Scoring: NAVIGATION ONLY");
        telemetry.addLine("VERIFY estimated poses and obstacle clearance first!");
        telemetry.addData("Start", "%.1f, %.1f, %.1f deg",
                follower.pose().x(), follower.pose().y(),
                Math.toDegrees(follower.pose().heading()));
        telemetry.update();
    }

    /** Starts the 30-second autonomous timer and first path. */
    @Override
    public void start() {
        autoTimer.reset();
        stateTimer.reset();
        logTimer.reset();
        started = true;
        changeState(AutoState.INITIAL_APPROACH_Path);
    }

    /** Runs Pedro, checks localization, advances states, and reports telemetry. */
    @Override
    public void loop() {
        if (!started || currentState == AutoState.FAULT) {
            showTelemetry();
            return;
        }

        follower.update();
        if (!validLocalization()) {
            fault("Pose discontinuity / invalid localization");
            showTelemetry();
            return;
        }

        if (logTimer.seconds() >= LOG_INTERVAL_SECONDS) {
            logRobotStatus("PERIODIC");
            logTimer.reset();
        }

        boolean parking = currentState == AutoState.DRIVE_TO_PARK_Path
                || currentState == AutoState.EMERGENCY_PARK_Path
                || currentState == AutoState.FINISHED;

        if (!parking && autoTimer.seconds() >= AUTO_SECONDS - PARK_RESERVE_SECONDS) {
            changeState(AutoState.EMERGENCY_PARK_Path);
        }

        switch (currentState) {
            case INITIAL_APPROACH_Path:
                if (arrived()) changeState(AutoState.WAIT_FOR_SCORING_SCORABLE);
                break;
            case WAIT_FOR_SCORING_SCORABLE:
                // Navigation-only: simulated CELL status; not real vision.
                if (stateTimer.seconds() >= SIMULATED_SCORABLE_WAIT_SECONDS) {
                    changeState(AutoState.SHOOT_FIRST);
                }
                break;
            case SHOOT_FIRST:
                if (stateTimer.seconds() >= TEST_SHOOT) {
                    changeState(AutoState.DRIVE_TO_FLOWER_Path);
                }
                break;
            case DRIVE_TO_FLOWER_Path:
                if (arrived()) changeState(AutoState.COLLECT_FLOWER);
                break;
            case COLLECT_FLOWER:
                if (stateTimer.seconds() >= TEST_COLLECTION) {
                    changeState(AutoState.FLOWER_TO_SCORING_Path);
                }
                break;
            case FLOWER_TO_SCORING_Path:
                if (arrived()) changeState(AutoState.SHOOT_SECOND);
                break;
            case SHOOT_SECOND:
                if (stateTimer.seconds() >= TEST_SHOOT) {
                    changeState(AutoState.DRIVE_TO_PARK_Path);
                }
                break;
            case DRIVE_TO_PARK_Path:
            case EMERGENCY_PARK_Path:
                if (parkingPathStarted && arrived()) changeState(AutoState.FINISHED);
                break;
            default:
                break;
        }
        showTelemetry();
    }

    /**
     * Changes currentState, starting the corresponding navigation path when required.
     * @param nextState next currentState
     */
    private void changeState(AutoState nextState) {
        if (currentState == AutoState.FAULT
                || currentState == AutoState.FINISHED)
            return;
        if ((currentState == AutoState.DRIVE_TO_PARK_Path
                || currentState == AutoState.EMERGENCY_PARK_Path)
                && nextState != AutoState.FINISHED)
            return;

        logPose("EXIT_TO_" + nextState);
        currentState = nextState;
        stateTimer.reset();
        pathNotBusySince = -1.0;
        activeTarget = null;
        RobotLog.ii("RedScoringAuto", "t=%.2f STATE=%s", autoTimer.seconds(), currentState);
        switch (nextState) {
            case INITIAL_APPROACH_Path:
                follow("Initial Approach Path",
                        paths.initialApproach,
                        paths.scoringPose);
                break;
            case DRIVE_TO_FLOWER_Path:
                follow("Drive to Flower",
                        paths.driveToFlower,
                        paths.collectionPose);
                break;
            case FLOWER_TO_SCORING_Path:
                follow("Flower to Scoring",
                        paths.flowerToScoring,
                        paths.scoringPose);
                break;
            case DRIVE_TO_PARK_Path:
                parkingPathStarted = true;
                follow("Drive to Park",
                        paths.scoringToPark,
                        paths.parkPose);
                break;
            case EMERGENCY_PARK_Path:
                navigation.cancel();
                follower.stop();
                parkingPathStarted = true;
                Pose current = follower.pose();

                follow("Emergency Park",
                        paths.emergencyPark(current), paths.parkPose);
                break;
            case FINISHED:
                navigation.cancel();
                follower.stop();
                break;
            default:
                break;
        }
    }

    /**
     * Begins a Pedro path and remembers its target for independent verification.
     * @param path path to follow
     * @param target expected final pose
     */
    //private void follow(Path path, Pose target) {
    //    activeTarget = target;
    //    follower.follow(path);
    //}
    //use the upldated follow function in PathCompletionRecovery class
    private void follow(String name,Path path, Pose target){
        navigation.begin(name, path, target, true,
                (currentPose,targetPose)->
                        paths.recoverPath(name,currentPose,targetPose));
    }

    /**
     * Verifies both follower completion and arrival near the target.
     * A follower that becomes idle far from its target is treated as a fault.
     * @return true when the robot has arrived within tolerance
     */
    private boolean arrived() {

        boolean parking = currentState == AutoState.DRIVE_TO_PARK_Path||
                currentState == AutoState.EMERGENCY_PARK_Path;
        double deadline = parking ? AUTO_SECONDS : AUTO_SECONDS - PARK_RESERVE_SECONDS;

        double remaining = Math.max(0.0, deadline - autoTimer.seconds());

        PathCompletionRecovery.Result result = navigation.update(remaining);
        switch (result) {
            case ARRIVED:return true;
            case RECOVERING:
                RobotLog.ii("RedScoringAuto",
                        "Navigation recovery attemp %d",
                        navigation.getRetries());
                return false;
            case FAILED:
                    fault(navigation.getFailureReason());
                    return false;
            case RUNNING:
            default:
                return false;
        }
    }

    /**
     * Detects gross discontinuities in the Pedro pose, such as a Pinpoint
     * read failure causing an unexpected (0,0,0) pose.
     * @return true if the pose is finite and continuous
     */
    private boolean validLocalization() {
        Pose now = follower.pose();
        if (now == null || !Double.isFinite(now.x()) ||
                !Double.isFinite(now.y()) || !Double.isFinite(now.heading())) return false;
        if (previousPose != null) {
            double distance = Math.hypot(now.x() - previousPose.x(),
                    now.y() - previousPose.y());
            double angle = Math.abs(Math.toDegrees(angleDifference(
                    now.heading(), previousPose.heading())));
            if (distance > MAX_POSITION_JUMP_INCHES || angle > MAX_HEADING_JUMP_DEGREES) {
                RobotLog.ee("RedScoringAuto",
                        "LOCALIZATION_JUMP dx=%.2f headingDelta=%.1f", distance, angle);
                return false;
            }
        }
        previousPose = now;
        return true;
    }

    /**
     * Returns the shortest signed angular difference in radians.
     * @param a first angle
     * @param b second angle
     * @return signed angular difference in [-pi, pi]
     */
    private static double angleDifference(double a, double b) {
        return Math.atan2(Math.sin(a - b), Math.cos(a - b));
    }

    /**
     * Immediately stops the follower on a detected safety fault.
     * @param reason explanation included in RobotLog and telemetry
     */
    private void fault(String reason) {
        faultReason = reason;
        RobotLog.ee("RedScoringAuto", "t=%.2f FAULT: %s", autoTimer.seconds(), reason);
        follower.stop();
        currentState = AutoState.FAULT;
        activeTarget = null;
    }

    /**
     * Writes pose, follower status, and currentState to the robot log.
     * @param event event description
     */
    private void logPose(String event) {
        if (follower == null) return;
        Pose pose = follower.pose();
        RobotLog.ii("RedScoringAuto",
                "t=%.2f event=%s currentState=%s x=%.2f y=%.2f heading=%.1f busy=%b",
                autoTimer.seconds(), event, currentState, pose.x(), pose.y(),
                Math.toDegrees(pose.heading()), follower.isBusy());
    }

    /** Displays current currentState, time, localization and fault reason. */
    private void showTelemetry() {
        telemetry.addData("State", currentState);
        telemetry.addData("Auto seconds", "%.2f / %.0f", autoTimer.seconds(), AUTO_SECONDS);
        telemetry.addData("Remaining", "%.2f", Math.max(0, AUTO_SECONDS - autoTimer.seconds()));
        telemetry.addData("Navigation only", NAVIGATION_ONLY);
        telemetry.addData("Fault", faultReason);
        if (follower != null) {
            Pose pose = follower.pose();
            telemetry.addData("X", "%.2f", pose.x());
            telemetry.addData("Y", "%.2f", pose.y());
            telemetry.addData("Heading deg", "%.1f", Math.toDegrees(pose.heading()));
            telemetry.addData("Path busy", follower.isBusy());
        }
        telemetry.update();
    }

    /** Stops the Pedro follower when the OpMode is stopped. */
    @Override
    public void stop() {
        started = false;
        if (follower != null) follower.stop();
    }
}

