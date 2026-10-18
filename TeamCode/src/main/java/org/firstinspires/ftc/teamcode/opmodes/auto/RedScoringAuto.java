package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.auto.DriveDiagnostics;
import org.firstinspires.ftc.teamcode.pedro.auto.PathCompletionRecovery;
import org.firstinspires.ftc.teamcode.pedro.auto.PathUtility;
import org.firstinspires.ftc.teamcode.pedro.auto.RedScoringPaths;

import java.util.Locale;

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
    /**
     * Possible autonomous states for Red Scoring Quarter
     */
    private enum RSAutoState implements PathUtility.AutoState {
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

    /**
     * A: stop follower, continue follower.update()
     * B: stop follower, skip follower.update() during stationary states
     * For shaking problem
     */
    private static final boolean TEST_A = true;//A
    private static final boolean testDrive = true;
    private DriveDiagnostics driveDiagnostics;

    private static final String LOG_TAG ="RedScoringAuto";
    private static final double AUTO_SECONDS = 30.0;
    private static final double PARK_RESERVE_SECONDS = 3.0;
    private static final double TEST_SHOOT = 1.0;
    private static final double TEST_COLLECTION = 2.0;
    private static final double SIMULATED_SCORABLE_WAIT_SECONDS = 5.0;
    private static final double FLOWER_TO_SCORING_WAIT_SECONDS = 3.0;
    private static final double DRIVE_TO_PARK_WAIT_SECONDS = 3.0;
    private static final double DRIVE_TO_FLOWER_WAIT_SECONDS = 3.0;

    private static final double LOG_INTERVAL_SECONDS = 0.10;

    //for task completion, recovery and idle checking
    private static final double POSITION_TOLERANCE_INCHES = 3.0;
    private static final double HEADING_TOLERANCE_DEGREES = 15.0;
    private static final double IDLE_GRACE_SECONDS=0.25;
    private static final int MAX_RETRIES=3;
    private static final double NO_PROGRESS_TIMEOUT_SECONDS=2.0;
    private static final double MAX_ATTEMPTS_SECONDS=5.0;
    private static final double MIN_PROGRESS_INCHES=1;


    private static final double MAX_POSITION_JUMP_INCHES = 12.0;
    private static final double MAX_HEADING_JUMP_DEGREES = 100.0;

    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime logTimer = new ElapsedTime();
    private Follower follower;
    private RSAutoState currentState;
    private RedScoringPaths paths;
    private boolean parkingPathStarted;
    //private AutoState currentState = AutoState.READY;
    private Pose previousPose;
    private boolean started;
    private boolean finished;
    private String faultReason = "";

    /**
     * detect the stationary states
     */
    private boolean isStationaryState(){
        switch (currentState){
            case WAIT_FOR_SCORING_SCORABLE:
            case SHOOT_FIRST:
            case COLLECT_FLOWER:
            case SHOOT_SECOND:
                return true;
            default:
                return false;
        }
    }

    /*
    *check whether the robt starts Pathing state
     */
    private boolean isParkingState(){
         return currentState == RSAutoState.DRIVE_TO_PARK_Path ||
                currentState == RSAutoState.EMERGENCY_PARK_Path;
    }

    /**
     * to stop the follower if the state is stationary.
     * To detect whether the robot shaking
     */
    private void stopForStationaryState(String event){
        Pose previous = follower.pose();
        if (testDrive){
            driveDiagnostics.log(autoTimer.seconds(),
                    currentState.name(),
                    event+"Before Stop",follower);
        }

        follower.stop();
        Pose after = follower.pose();

        if (testDrive){
            driveDiagnostics.log(autoTimer.seconds(),
                    currentState.name(),
                    event+"After Stop",follower);
        }

        if (testDrive){
            driveDiagnostics.logPoseDifference(autoTimer.seconds(),
                    currentState.name(),previous,after);
        }

        PathUtility.logPose(LOG_TAG,event,follower,autoTimer,currentState);
    }

    /**
     * Begin a path
     */
    private void beginPath(String name, Path path,Pose target){
        PathUtility.info(LOG_TAG,PathUtility.pathStart(autoTimer.seconds(),name,target));
        follower.follow(path);
        if (testDrive){
            driveDiagnostics.log(autoTimer.seconds(),
                    currentState.name(),
                    name+"Path started",follower);
        }
    }

    /** Initializes Pedro localization and the navigation-only test. */
    @Override
    public void init() {
        if (!NAVIGATION_ONLY) {
            throw new IllegalStateException("Mechanisms and Vision are not implemented");
        }
        paths = new RedScoringPaths();
        follower = Constants.create(hardwareMap);
        if (testDrive){
            driveDiagnostics = new DriveDiagnostics(hardwareMap,
                    "leftFront","rightFront",
                    "leftBack","rightBack",LOG_TAG);
        }

        follower.setPose(paths.startPose);
        previousPose = follower.pose();
        currentState = RSAutoState.READY;
        started = false;
        parkingPathStarted = false;
        telemetry.addLine("Red Scoring: NAVIGATION ONLY");
        telemetry.addLine("VERIFY estimated poses and obstacle clearance first!");
        //telemetry.addData("Start", "%.1f, %.1f, %.1f deg",
        //        follower.pose().x(), follower.pose().y(),
        //        Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Start",
                "pose= %s", PathUtility.formatPose(follower.pose()));
        telemetry.update();
    }

    /** Starts the 30-second autonomous timer and first path. */
    @Override
    public void start() {
        autoTimer.reset();
        stateTimer.reset();
        logTimer.reset();
        started = true;
        changeState(RSAutoState.INITIAL_APPROACH_Path,"Start Auto");
        PathUtility.info(LOG_TAG,"Diagnostic_Test= %s"+(TEST_A?"A":"B"));
    }

    /** Runs Pedro, checks localization, advances states, and reports telemetry. */
    @Override
    public void loop() {
        if (!started || currentState == RSAutoState.FAULT||currentState==RSAutoState.FINISHED) {
            showTelemetry();
            return;
        }

        if (autoTimer.seconds()>=AUTO_SECONDS){
            changeState(RSAutoState.FINISHED,"30 seconds due");
            showTelemetry();
            return;
        }

        //only update while the robot is doing pathing
        //follower.update();
        if (TEST_A || !isStationaryState()){
            follower.update();
        }

        if (!validLocalization()) {
            fault("Pose discontinuity / invalid localization");
            showTelemetry();
            return;
        }

        if (logTimer.seconds() >= LOG_INTERVAL_SECONDS) {
            if (testDrive){
                String event = TEST_A? "Test A": "Test B";
                driveDiagnostics.log(autoTimer.seconds(),
                        currentState.name(),
                        event+"Periodic",follower);
            }
            logTimer.reset();
        }

        if (!isParkingState() && autoTimer.seconds() >= AUTO_SECONDS - PARK_RESERVE_SECONDS) {
            PathUtility.warning(LOG_TAG,PathUtility.deadline(autoTimer.seconds(),
                    AUTO_SECONDS - autoTimer.seconds(),PARK_RESERVE_SECONDS));
            changeState(RSAutoState.EMERGENCY_PARK_Path,"Parking Time Reserved");
            showTelemetry();
            return;
        }

        double elapsed = stateTimer.seconds();
        boolean pathComplete = elapsed>=0.05 && !follower.isBusy();
        switch (currentState) {
            case INITIAL_APPROACH_Path:
                if (pathComplete) changeState(RSAutoState.WAIT_FOR_SCORING_SCORABLE,"Arrived at Scoring Position");
                break;
            case WAIT_FOR_SCORING_SCORABLE:
                // Navigation-only: simulated CELL status; not real vision.
                if (stateTimer.seconds() >= SIMULATED_SCORABLE_WAIT_SECONDS) {
                    changeState(RSAutoState.SHOOT_FIRST,"The cell is ready");
                }
                break;
            case SHOOT_FIRST:
                if (stateTimer.seconds() >= TEST_SHOOT) {
                    changeState(RSAutoState.DRIVE_TO_FLOWER_Path,"The shoot is complete");
                }
                break;
            case DRIVE_TO_FLOWER_Path:
                if (pathComplete) changeState(RSAutoState.COLLECT_FLOWER,"Arrived at Flower");
                break;
            case COLLECT_FLOWER:
                if (stateTimer.seconds() >= TEST_COLLECTION) {
                    changeState(RSAutoState.FLOWER_TO_SCORING_Path,"The flower is collected");
                }
                break;
            case FLOWER_TO_SCORING_Path:
                if (pathComplete) changeState(RSAutoState.SHOOT_SECOND, "Arrived at Scoring ");
                break;
            case SHOOT_SECOND:
                if (stateTimer.seconds() >= TEST_SHOOT) {
                    changeState(RSAutoState.DRIVE_TO_PARK_Path,"The second shoot is complete");
                }
                break;
            case DRIVE_TO_PARK_Path:
            case EMERGENCY_PARK_Path:
                if (parkingPathStarted && pathComplete)
                    changeState(RSAutoState.FINISHED,"arrived");
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
    private void changeState(RSAutoState nextState, String reason) {
        if (currentState == RSAutoState.FAULT
                || currentState == RSAutoState.FINISHED)
            return;
        if (isParkingState() && nextState != RSAutoState.FINISHED) return;

        PathUtility.info(LOG_TAG, PathUtility.stateChange(
                autoTimer.seconds(),currentState,nextState,reason));

        currentState = nextState;
        stateTimer.reset();

        if (isStationaryState()){
            stopForStationaryState("Stationary State Entry");
        }

        switch (nextState) {
            case INITIAL_APPROACH_Path:
                beginPath("Initial Approach Path",
                        paths.initialApproach,
                        paths.scoringPose);
                break;
            case DRIVE_TO_FLOWER_Path:
                beginPath("Drive to Flower",
                        paths.driveToFlower,
                        paths.collectionPose);
                break;
            case FLOWER_TO_SCORING_Path:
                beginPath("Flower to Scoring",
                        paths.flowerToScoring,
                        paths.scoringPose);
                break;
            case DRIVE_TO_PARK_Path:
                parkingPathStarted = true;
                beginPath("Drive to Park",
                        paths.scoringToPark,
                        paths.parkPose);
                break;
            case EMERGENCY_PARK_Path:
                stopForStationaryState("Emergency Parking");
                parkingPathStarted = true;
                Pose current = follower.pose();
                if (current == null) {
                    fault("No pose for emergency parking");
                    return;
                }
                beginPath("Emergency Park",
                        paths.emergencyPark(current), paths.parkPose);
                break;
            case FINISHED:
                if(testDrive){
                    driveDiagnostics.log(autoTimer.seconds(),
                            currentState.name(),
                            "In Finished, before calling stop",follower);
                }
                stopForStationaryState("Finished");
                follower.update();//added to stop motor
                if (testDrive){
                    driveDiagnostics.log(autoTimer.seconds(),
                            currentState.name(),
                            "In Finished, after calling stop",follower);
                    driveDiagnostics.stopAllMotors();
                }

                break;
            default:
                break;
        }
    }


    /**
     * Verifies both follower completion and arrival near the target.
     * A follower that becomes idle far from its target is treated as a fault.
     * @return true when the robot has arrived within tolerance
     */
    /*
    private boolean arrived() {

        boolean parking = currentState == RSAutoState.DRIVE_TO_PARK_Path||
                currentState == RSAutoState.EMERGENCY_PARK_Path;
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
*/
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
            double angle = Math.abs(Math.toDegrees(PathUtility.angleDifference(
                    now.heading(), previousPose.heading())));
            if (distance > MAX_POSITION_JUMP_INCHES || angle > MAX_HEADING_JUMP_DEGREES) {
                RobotLog.ee(LOG_TAG,
                        "LOCALIZATION_JUMP dx=%.2f headingDelta=%.1f", distance, angle);
                return false;
            }
        }
        previousPose = now;
        return true;
    }



    /**
     * Immediately stops the follower on a detected safety fault.
     * @param reason explanation included in RobotLog and telemetry
     */
    private void fault(String reason) {
        faultReason = reason;
        PathUtility.error(LOG_TAG,"Fault"+reason);
        if (follower!=null) stopForStationaryState("Fault");
        currentState = RSAutoState.FAULT;
    }


    /** Displays current currentState, time, localization and fault reason. */
    private void showTelemetry() {
        telemetry.addData("State", currentState);
        telemetry.addData("Auto seconds", "%.2f / %.0f", autoTimer.seconds(), AUTO_SECONDS);
        telemetry.addData("Time Elapsed","%.2f",stateTimer.seconds());
        telemetry.addData("Remaining", "%.2f", Math.max(0, AUTO_SECONDS - autoTimer.seconds()));
        telemetry.addData("Navigation only", NAVIGATION_ONLY);
        telemetry.addData("Fault", faultReason);
        if (follower != null) {
            Pose pose = follower.pose();
            telemetry.addData("Pose: ",PathUtility.formatPose(pose));
            //telemetry.addData("X", "%.2f", pose.x());
            //telemetry.addData("Y", "%.2f", pose.y());
            //telemetry.addData("Heading deg", "%.1f", Math.toDegrees(pose.heading()));
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

