
package org.firstinspires.ftc.teamcode.opmodes.auto;


import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.auto.DriveDiagnostics;
import org.firstinspires.ftc.teamcode.pedro.auto.PathUtility;
import org.firstinspires.ftc.teamcode.pedro.auto.RedAudiencePaths;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import org.firstinspires.ftc.teamcode.pedro.auto.PathCompletionRecovery;

import java.util.Locale;

/**
 * Autonomous routine for one Red alliance robot operating in the
 * Red Audience quarter of the BIOBUZZ field.
 *
 * <p>Sequence:
 * <ol>
 *   <li>Shoot preloaded POLLEN into the Red Audience CELL.</li>
 *   <li>Drive to the Red Garden and collect up to four POLLEN.</li>
 *   <li>Return to the Red Audience shooting position.</li>
 *   <li>Wait until the CELL is scorable, then shoot.</li>
 *   <li>Drive to the nearby Flower and collect up to four POLLEN.</li>
 *   <li>Return to the Audience shooting position and shoot.</li>
 *   <li>Park before the 30-second autonomous period ends.</li>
 * </ol>
 *
 * <p>This initial implementation supports a navigation-only test mode.
 * Hardware-specific methods must be connected before live operation.
 */
@Autonomous(name = "Red Audience Auto - State Machine", group = "Autonomous")
public class RedAudienceAuto extends OpMode {

    /**
     * States used exclusively by this autonomous routine.
     */
    private enum RAAutoState implements PathUtility.AutoState {
        READY,//the robot is ready
        INITIAL_APPROACH_Path,//from start pose to shoot pose
        INITIAL_SHOOT, //shooting
        DRIVE_TO_GARDEN_Path, //from shoot pose to garden pose
        COLLECT_GARDEN,  //collecting from garden

        GARDEN_TO_AUDIENCE_Path,//from garden pose to shoot pose
        WAIT_FOR_AUDIENCE_SCORABLE,//waiting for audience to be ready
        SHOOT_GARDEN_POLLEN,//shooting four pollens collected from garden

        DRIVE_TO_FLOWER_Path,//from shoot pose to flower pose
        COLLECT_FLOWER, //collecting from flower

        FLOWER_TO_AUDIENCE_Path,//from flower pose to shoot pose
        CONFIRM_AUDIENCE_SCORABLE,//should be a check
        SHOOT_FLOWER_POLLEN,//shooting four pollens

        DRIVE_TO_PARK_Path,//from shoot pose to park pose
        EMERGENCY_PARK_Path,//from current pose to park pose
        FINISHED,  //finish the whole process,
        FAULT
    }

    /**
     * Set true for testing navigation without activating the
     * intake or shooting mechanism.
     */
    private static final boolean NAVIGATION_ONLY = true;
    private static final String LOG_TAG = "RedAudienceAuto";

    private final Vision cam = Robot.INSTANCE.vision;
    private boolean audienceCellScorable = false;
    private String audienceCellStatus = "Not checked";
    //only when audience cell is ready again and all pollens are shot
    private static boolean ENABLE_FLOWER_CYCLE= true;//test only the first part shoot and garden.

    private static final boolean testDrive = true;
    //total duration period
    private static final double AUTO_SECONDS = 30.0;
    private static final double SAFETY_MARGIN_SECONDS=1.0;
    private static final double LOG_INTERVAL_SECONDS=0.1;
    /**
     * Reserve time for the final parking movement.
     * Replace this estimate with a measured value.
     */
    private static final double PARK_RESERVE_SECONDS = 5.0;

    /**
     * Temporary collection duration for navigation-only testing.
     * This is not a substitute for a full-capacity sensor.
     */
    private static final double TEST_COLLECTION = 2.0;
    private static final double MAX_COLLECTION_TIME = 3.0;
    private static final double DRIVE_TO_FLOWER_TIME = 2.0;
    /**
     * Temporary shooting duration for navigation-only testing.
     */
    private static final double TEST_SHOOT = 1.0;

    private static final double FLOWER_CYCLE=2*DRIVE_TO_FLOWER_TIME +
            TEST_COLLECTION + TEST_SHOOT; //FULL CYCLE

    //for testing, wait for audience cell to be ready 2-3 seconds
    private static final double AUDIENCE_WAIT = 2.0;
    private static final double MAX_AUDIENCE_WAIT = 3.0;

    private static final double MAX_POSITION_JUMP_INCHES = 12.0;
    private static final double MAX_HEADING_JUMP_DEGREES = 100.0;

    private static final boolean TEST_A = true;//A
    private Follower follower;
    private DriveDiagnostics driveDiagnostics;
    private RAAutoState currentState;
    private RedAudiencePaths paths;
    private boolean emergencyParkStarted;
    private boolean parkingPathStarted;
    private boolean finished;
    private boolean started;

    private Pose previousPose;
    private String lastTransition ="Not Started";
    private String transitionReason="";
    private String faultReason="";

    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime logTimer = new ElapsedTime();


    @Override
    public void init() {
        if(!NAVIGATION_ONLY){
            throw new IllegalStateException("Connect shooter, vision, and intake adapters before live auto");
        }

        follower = Constants.create(hardwareMap);
        cam.init(hardwareMap,telemetry);//camera
        PathUtility.info(LOG_TAG,"Vision initialized");
        paths = new RedAudiencePaths();
        driveDiagnostics = new DriveDiagnostics(hardwareMap,
                "leftFront","rightFront",
                "leftBack","rightBack",LOG_TAG);

        if (follower != null) {
            follower.setPose(paths.startPose);
        }

        previousPose = follower.pose();
        currentState = RAAutoState.READY;

        started = false;
        parkingPathStarted = false;

        telemetry.addLine("Red Audience Auto initialized");
        telemetry.addData("Navigation only", NAVIGATION_ONLY);
        telemetry.addLine("Configure follower and field poses before running.");
        telemetry.addData("Start X,Y","%.1f %.1f",follower.pose().x(),follower.pose().y());
        telemetry.addData("Start heading","%.1f",Math.toDegrees(follower.pose().heading()));
        telemetry.update();
    }

    @Override
    public void start() {
        autoTimer.reset();
        stateTimer.reset();
        logTimer.reset();
        started = true;
        PathUtility.info(LOG_TAG,"Diagnostic_Test= %s"+(TEST_A?"A":"B"));
        changeState(RAAutoState.INITIAL_APPROACH_Path,"Start Auto");
    }

    /**
     * Detect the stationary state
     */
    private boolean isStationaryState(){
        switch (currentState){
            case INITIAL_SHOOT:
            case COLLECT_FLOWER:
            case COLLECT_GARDEN:
            case WAIT_FOR_AUDIENCE_SCORABLE:
            case CONFIRM_AUDIENCE_SCORABLE:
            case SHOOT_GARDEN_POLLEN:
            case SHOOT_FLOWER_POLLEN:
                return true;
            default:
                return false;
        }
    }

    /**
     * check whether the robot starts parking
     */
    private boolean parking(){
        return currentState == RAAutoState.DRIVE_TO_PARK_Path ||
                currentState == RAAutoState.EMERGENCY_PARK_Path;
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
        currentState = RAAutoState.FAULT;
        driveDiagnostics.stopAllMotors();
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

    /*
     *check whether the robt starts Pathing state
     */
    private boolean isParkingState(){
        return currentState == RAAutoState.DRIVE_TO_PARK_Path ||
                currentState == RAAutoState.EMERGENCY_PARK_Path;
    }
    /**
     * Determines whether the robot must begin parking.
     *
     * @return true when the parking reserve has been reached
     */
    private boolean mustParkNow() {
        return autoTimer.seconds()
                >= AUTO_SECONDS - PARK_RESERVE_SECONDS;
    }

    @Override
    public void loop() {
        //already done the whole process
       if (!started || currentState == RAAutoState.FAULT
               || currentState == RAAutoState.FINISHED){
           showTelemetry();
           return;
       }

       //stop the follower is the state is stationary
        //only update when it is a path
       if (TEST_A ||!isStationaryState()) {
           follower.update();
       }

       //log trouble
       if(logTimer.seconds()>=LOG_INTERVAL_SECONDS){
           if (testDrive){
               String event = TEST_A? "Test A": "Test B";
               driveDiagnostics.log(autoTimer.seconds(),
                       currentState.name(),
                       event+"Periodic",follower);
           }
           logTimer.reset();
       }

       //check odom error
       if (!validLocalization()){
           fault("Pose discontinuity / invalid localization");
           showTelemetry();
           return;
       }

       //30 seconds run out
       if (autoTimer.seconds()>=AUTO_SECONDS){
           changeState(RAAutoState.FINISHED,"30 seconds due");
           showTelemetry();
           return;
       }

        /*
         * TODO: Update any subsystem that requires periodic updates,
         * including your existing Vision and Shooter implementations.
         */
        //let shooting flower pollen complete, even before emergency parking
        boolean protectedFlowerShooting = currentState == RAAutoState.SHOOT_FLOWER_POLLEN;

        //emergency parking
        if (!protectedFlowerShooting && !isParkingState() && mustParkNow()) {
            PathUtility.warning(LOG_TAG,PathUtility.deadline(autoTimer.seconds(),
                    AUTO_SECONDS-autoTimer.seconds(),PARK_RESERVE_SECONDS));
            changeState(RAAutoState.EMERGENCY_PARK_Path, "Parking Time Reserved");
            showTelemetry();
            return;
        }


        double elapsed = stateTimer.seconds();
        boolean pathComplete = elapsed>=0.05 && !follower.isBusy();

        switch (currentState) {
            case INITIAL_APPROACH_Path:
                if(pathComplete) changeState(RAAutoState.INITIAL_SHOOT,"Ready to Shoot");
                break;
            case INITIAL_SHOOT:
                if (shootingFinished()) {
                    changeState(RAAutoState.DRIVE_TO_GARDEN_Path,"Shoot Completed");
                }
                break;
            case DRIVE_TO_GARDEN_Path:
                if (pathComplete) {
                    changeState(RAAutoState.COLLECT_GARDEN,"Arrived at Garden");
                }
                break;
            case COLLECT_GARDEN:
                if (collectionFinished()) {
                    //stopCollectionGarden();
                    changeState(RAAutoState.GARDEN_TO_AUDIENCE_Path,"Garden Collection Completed.");
                }
                break;
            case GARDEN_TO_AUDIENCE_Path:
                if (pathComplete) {
                    changeState(RAAutoState.WAIT_FOR_AUDIENCE_SCORABLE,"Arrived at Shooting pose");
                }
                break;

            case WAIT_FOR_AUDIENCE_SCORABLE:
                if (isAudienceCellScorable()){
                    changeState(RAAutoState.SHOOT_GARDEN_POLLEN,"Audience Cell Ready");
                }else if (!hasTimeFor(TEST_SHOOT,PARK_RESERVE_SECONDS)){
                    changeState(RAAutoState.EMERGENCY_PARK_Path,"Audience Cell Not Ready, go park");
                }
                break;
            case SHOOT_GARDEN_POLLEN:
                if (shootingFinished()) {
                    if (canAttemptFlowerCycle()) {
                        changeState(RAAutoState.DRIVE_TO_FLOWER_Path,"Shooting garden pollen finished");
                    }else{
                        changeState(RAAutoState.EMERGENCY_PARK_Path, "no time for flower collection");
                    }
                }
                break;

            case DRIVE_TO_FLOWER_Path:
                if (pathComplete) {
                    changeState(RAAutoState.COLLECT_FLOWER,"Arrived at Flower");
                }
                break;

            case COLLECT_FLOWER:
                if (collectionFinished()) {
                    //stopFlowerCollection();
                    changeState(RAAutoState.FLOWER_TO_AUDIENCE_Path,"Collection Flower finished");
                }
                break;

            case FLOWER_TO_AUDIENCE_Path:
                if (pathComplete){
                    changeState(RAAutoState.CONFIRM_AUDIENCE_SCORABLE,"Arrived at Audience");
                }
                break;
            case CONFIRM_AUDIENCE_SCORABLE:
                if (isAudienceCellScorable()) {
                    if (hasTimeFor(TEST_SHOOT,PARK_RESERVE_SECONDS)) {
                        changeState(RAAutoState.SHOOT_FLOWER_POLLEN,"Audience Cell Ready for shooting");
                    }else{
                        changeState(RAAutoState.EMERGENCY_PARK_Path,"No time to shoot, emergency park");
                    }
                }
                break;
            case SHOOT_FLOWER_POLLEN:
                if (shootingFinished()) {
                    changeState(RAAutoState.DRIVE_TO_PARK_Path,"Finish last shooting, Go Park");
                }
                break;

            case DRIVE_TO_PARK_Path:
                if (parkingPathStarted && pathComplete) {
                    changeState(RAAutoState.FINISHED,"complete the auto");
                }
                break;
            case EMERGENCY_PARK_Path:
                if (parkingPathStarted && pathComplete) {
                    changeState(RAAutoState.FINISHED,"Complete the auto with emergency parking");
                }
                break;
            case FINISHED:
                break;
        }

        showTelemetry();
    }

    /**
     * Changes the autonomous state and resets its elapsed time.
     *
     * @param nextState state to execute on the next loop
     */
    private void changeState(RAAutoState nextState, String reason) {

        if (currentState == RAAutoState.FINISHED ||
        currentState== RAAutoState.FAULT) {return;}

        if(parking() &&nextState != RAAutoState.FINISHED
                && nextState!=RAAutoState.FAULT){

            return;//do not interrupt parking to earn points
        }

        PathUtility.info(LOG_TAG,
                PathUtility.stateChange(autoTimer.seconds(),currentState,
                        nextState,reason));
        //stopCollection();
        //stopShooting();

        currentState = nextState;
        stateTimer.reset();

        if(isStationaryState()){
            stopForStationaryState("Stationary State Entry");
        }

        switch(nextState){
            case INITIAL_APPROACH_Path:
                //follower.follow(paths.initialApproach);
                beginPath("Initial Approach",paths.initialApproach,
                        paths.shootPose);
                break;
            case INITIAL_SHOOT:
            case SHOOT_FLOWER_POLLEN:
            case SHOOT_GARDEN_POLLEN:
                beginShooting();
                break;
            case DRIVE_TO_GARDEN_Path:
                //follower.follow(paths.driveToGarden);
                beginPath("Drive to Garden",
                        paths.driveToGarden,paths.gardenPose);
                break;
            case COLLECT_FLOWER:
                beginCollectionFlower();
                break;
            case COLLECT_GARDEN:
                beginCollectionGarden();
                break;
            case GARDEN_TO_AUDIENCE_Path:
                //follower.follow(paths.gardenToAudience);
                beginPath("Garden to Audience",paths.gardenToAudience,
                        paths.shootPose);
                break;
            case DRIVE_TO_FLOWER_Path:
                //follower.follow(paths.driveToFlower);
                beginPath("Drive to Flower",paths.driveToFlower,paths.flowerPose);
                break;
            case FLOWER_TO_AUDIENCE_Path:
                //follower.follow(paths.flowerToAudience);
                beginPath("Flower to Audience",paths.flowerToAudience,paths.shootPose);
                break;
            case EMERGENCY_PARK_Path:
                //follower.stop();
                stopForStationaryState("Emergency Park");
                Pose current = follower.pose();
                if (current == null){
                    fault("No pose for emergency park");
                    return;
                }
                parkingPathStarted = true;
                beginPath("Emergency Park",
                        paths.emergencyPark(current),
                        paths.parkPose);
                //follower.follow(parkingPath);
                break;
            case DRIVE_TO_PARK_Path:

                parkingPathStarted = true;
                //follower.follow(paths.driveToPark);
                beginPath("Drive to Park",
                        paths.driveToPark,paths.parkPose);
                break;
            case WAIT_FOR_AUDIENCE_SCORABLE:
            //case WAIT_FOR_AUDIENCE_AGAIN:
            case CONFIRM_AUDIENCE_SCORABLE:
                PathUtility.info(LOG_TAG,"Waiting for scorable Audience cell"+nextState);
                PathUtility.logPose(LOG_TAG,"Cell Wait start",follower,autoTimer,currentState);

                break;
            case FINISHED:
                if(testDrive){
                    driveDiagnostics.log(autoTimer.seconds(),
                            currentState.name(),
                            "In finished, before calling stop",follower);
                }
                stopForStationaryState("Finished");
                follower.update();
                if(testDrive)
                {
                    driveDiagnostics.log(autoTimer.seconds(),
                            currentState.name(),
                            "In finished, after calling stop",follower);
                    driveDiagnostics.stopAllMotors();
                }
                finished = true;
                break;
            default:
                break;
        }
    }


    /**
     * Determines whether there is sufficient time to attempt
     * the optional Flower collection and shooting cycle.
     *
     * @return true if the optional cycle may be attempted
     */
    private boolean canAttemptFlowerCycle(){
        boolean restTask = hasTimeFor(FLOWER_CYCLE,PARK_RESERVE_SECONDS);
        return ENABLE_FLOWER_CYCLE && restTask;
    }
    /**
     * Starts the shooting operation.
     */
    private void beginShooting() {

        if (NAVIGATION_ONLY) {
            return;
        }

        /*
         * TODO: Connect your existing Shooter subsystem.
         * AprilTagCluster is detected as scorable
         * Aim turret using Vision CELL X/Y.
         * Spin up flywheel.
         * Feed POLLEN sequentially.
         * Stop if CELL becomes unscorable.
         */
    }

    /**
     * Checks whether the current shooting operation is complete.
     *
     * @return true when shooting is finished
     */
    private boolean shootingFinished() {

        if (NAVIGATION_ONLY) {
            return stateTimer.seconds() >= TEST_SHOOT;
        }

        /*
         * TODO: Query your actual shooting controller.
         * Do not use a fixed timer to assume four shots succeeded.
         */
        return false;
    }

    /**
     * Stops the shooting mechanism.
     */
    private void stopShooting() {
        if (NAVIGATION_ONLY) return;
        /*
         * TODO: Stop the feeder and flywheel safely.
         */
    }

    /**
     * Starts the POLLEN intake from Garden.
     */
    private void beginCollectionGarden() {

        if (NAVIGATION_ONLY) {
            return;
        }

        /*
         * TODO: Activate the intake mechanism.
         */
    }

    /**
     * Starts the POLLEN intake from Flower.
     */
    private void beginCollectionFlower() {

        if (NAVIGATION_ONLY) {
            return;
        }

        /*
         * TODO: Activate the intake mechanism.
         */
    }

    /**
     * Checks whether collection has finished.
     *
     * @return true when full or a collection timeout is reached
     */
    private boolean collectionFinished() {

        if (NAVIGATION_ONLY) {
            return stateTimer.seconds() >= TEST_COLLECTION;
        }

        /*
         * TODO: Return true when the robot is full
         * (maximum four POLLEN) or the collection timeout expires.
         */
        return stateTimer.seconds()>=MAX_COLLECTION_TIME;
    }

    /**
     * Stops the POLLEN intake.
     */
    private void stopCollection() {
        if (NAVIGATION_ONLY) return;
        /*
         * TODO: Stop the intake motor.
         */
    }

    /**
     * Use the current Vision AprilTagCluster detection to confirm the
     * Red Audience cell is scorable.
     * A missing detection is treated as not scorable.
     * @return true if the cell is scorable
     */
    private boolean isAudienceCellScorable(){
        //will change to vision detection
        //return stateTimer.seconds() >= SIMULATED_SCORABLE_WAIT_SECONDS;
        audienceCellScorable = false;
        audienceCellStatus = "No scorable red scoring cluster";
        for(AprilTagDetection detection: cam.getDetections()){
            if(!(detection instanceof AprilTagClusterDetection)) continue;
            AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
            if(cluster.metadata == null || cluster.metadata.name == null) continue;
            String name = cluster.metadata.name.toUpperCase(Locale.ROOT);
            if(!name.contains("RED") || !name.contains("AUDIENCE")) continue;
            if(cam.isCellScorable(cluster)){
                audienceCellScorable = true;
                audienceCellStatus = cluster.metadata.name+" roll"
                        +String.format(Locale.US, "%.1f", cluster.ftcPose.roll);
                break;
            }
            audienceCellStatus = cluster.metadata.name+" detected, not scorable";
        }
        if (audienceCellScorable == true){
            PathUtility.info(LOG_TAG,
                    String.format(Locale.US,"Red Audience Cell: detected%b, scorable=%b, cluster=%s",
                            true,true,"Red Audience"));
        }
        return audienceCellScorable;
    }

    /**
     * Checks whether the next task can be completed while
     * preserving enough time for parking.
     */
    private boolean hasTimeFor(double taskSeconds, double parkSeconds){
        double remaining = AUTO_SECONDS - autoTimer.seconds();
        return remaining >= taskSeconds + parkSeconds+SAFETY_MARGIN_SECONDS;
    }

    @Override
    public void stop() {
        stopCollection();
        stopShooting();

        if (follower != null) {
            follower.stop();
        }
        if (driveDiagnostics!=null){
            driveDiagnostics.stopAllMotors();
        }
        cam.close();
        PathUtility.info(LOG_TAG,"Auto stopped, camera closed, drivetrain stopped.");
    }
}

