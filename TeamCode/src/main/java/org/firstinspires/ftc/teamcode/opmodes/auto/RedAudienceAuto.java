
package org.firstinspires.ftc.teamcode.opmodes.auto;


import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.auto.RedAudiencePaths;

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
    private enum AutoState {
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
        FINISHED  //finish the whole process
    }

    /**
     * Set true for testing navigation without activating the
     * intake or shooting mechanism.
     */
    private static final boolean NAVIGATION_ONLY = true;

    //only when audience cell is ready again and all pollens are shot
    private static boolean ENABLE_FLOWER_CYCLE= false;//test only the first part shoot and garden.

    //total duration period
    private static final double AUTO_DURATION_SECONDS = 30.0;
    private static final double SAFETY_MARGIN_SECONDS=1.0;
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


    private Follower follower;

    private AutoState currentState;
    private RedAudiencePaths paths;
    private boolean emergencyParkStarted;
    private boolean parkingPathStarted;
    private boolean finished;
    private boolean started;

    private AutoState previousState;
    private String lastTransition ="Not Started";
    private String transitionReason="";

    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();

    @Override
    public void init() {
        if(!NAVIGATION_ONLY){
            throw new IllegalStateException("Connect shooter, vision, and intake adapters before live auto");
        }

        follower = Constants.create(hardwareMap);
        paths = new RedAudiencePaths();

        if (follower != null) {
            follower.setPose(paths.startPose);
        }

        currentState = AutoState.READY;
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
        started = true;
        changeState(AutoState.INITIAL_APPROACH_Path);
    }

    @Override
    public void loop() {

       if (!started){
           return;
       }
       follower.update();

        /*
         * TODO: Update any subsystem that requires periodic updates,
         * including your existing Vision and Shooter implementations.
         */

        boolean parking = currentState == AutoState.DRIVE_TO_PARK_Path
                || currentState == AutoState.EMERGENCY_PARK_Path
                || currentState == AutoState.FINISHED;
        boolean protectedFlowerShooting = currentState == AutoState.SHOOT_FLOWER_POLLEN;

        if (mustParkNow()
                && !parking
                && !protectedFlowerShooting) {

            changeState(AutoState.EMERGENCY_PARK_Path);
        }


        switch (currentState) {
            case INITIAL_APPROACH_Path:
                if(pathFinished()) changeState(AutoState.INITIAL_SHOOT);
                break;
            case INITIAL_SHOOT:
                if (shootingFinished()) {
                    changeState(AutoState.DRIVE_TO_GARDEN_Path);
                }
                break;

            case DRIVE_TO_GARDEN_Path:
                if (pathFinished()) {
                    changeState(AutoState.COLLECT_GARDEN);
                }
                break;
            case COLLECT_GARDEN:
                if (collectionFinished()) {
                    //stopCollectionGarden();
                    changeState(AutoState.GARDEN_TO_AUDIENCE_Path);
                }
                break;

            case GARDEN_TO_AUDIENCE_Path:
                if (pathFinished()) {
                    changeState(AutoState.WAIT_FOR_AUDIENCE_SCORABLE);
                }
                break;

            case WAIT_FOR_AUDIENCE_SCORABLE:
                if (!hasTimeFor(TEST_SHOOT,PARK_RESERVE_SECONDS)){
                    changeState(AutoState.EMERGENCY_PARK_Path);
                }
                else if (isAudienceCellScorable()) {
                    changeState(AutoState.SHOOT_GARDEN_POLLEN);
                }
                break;

            case SHOOT_GARDEN_POLLEN:
                if (shootingFinished()) {

                    if (canAttemptFlowerCycle()) {
                        changeState(AutoState.DRIVE_TO_FLOWER_Path);
                    }else{
                        changeState(AutoState.EMERGENCY_PARK_Path);
                    }
                }
                break;

            case DRIVE_TO_FLOWER_Path:
                if (pathFinished()) {
                    changeState(AutoState.COLLECT_FLOWER);
                }
                break;

            case COLLECT_FLOWER:
                if (collectionFinished()) {
                    //stopFlowerCollection();
                    changeState(AutoState.FLOWER_TO_AUDIENCE_Path);
                }
                break;

            case FLOWER_TO_AUDIENCE_Path:
                if (pathFinished()){
                    changeState(AutoState.CONFIRM_AUDIENCE_SCORABLE);
                }
                break;
            case CONFIRM_AUDIENCE_SCORABLE:
                if (isAudienceCellScorable()) {
                    if (hasTimeFor(TEST_SHOOT,PARK_RESERVE_SECONDS)) {
                        changeState(AutoState.SHOOT_FLOWER_POLLEN);
                    }else{
                        changeState(AutoState.EMERGENCY_PARK_Path);
                    }
                } else if(stateTimer.seconds()>=MAX_AUDIENCE_WAIT){
                    changeState(AutoState.EMERGENCY_PARK_Path);
                }
                break;

            case SHOOT_FLOWER_POLLEN:
                if (shootingFinished()) {
                    changeState(AutoState.DRIVE_TO_PARK_Path);
                }
                break;

            case DRIVE_TO_PARK_Path:
                if (parkingPathStarted && pathFinished()) {
                    changeState(AutoState.FINISHED);
                }
                break;
            case EMERGENCY_PARK_Path:
                if (parkingPathStarted && pathFinished()) {
                    changeState(AutoState.FINISHED);
                }
                break;
            case FINISHED:
                break;
        }

        telemetry.addData("State", currentState);
        telemetry.addData("Last transition", lastTransition);
        telemetry.addData("Transition reason", transitionReason);
        telemetry.addData("state time", "%.1f", stateTimer.seconds());
        telemetry.addData("Auto time", "%.1f", autoTimer.seconds());
        telemetry.addData("Remaining", "%.1f",
                Math.max(0, AUTO_DURATION_SECONDS - autoTimer.seconds()));
        telemetry.addData("Path busy", follower.isBusy());
        telemetry.addData("X", "%.1f", follower.pose().x());
        telemetry.addData("Y", "%.1f", follower.pose().y());
        telemetry.addData("Heading", "%.1f", Math.toDegrees(follower.pose().heading()));
        telemetry.update();
    }

    /**
     * Changes the autonomous state and resets its elapsed time.
     *
     * @param nextState state to execute on the next loop
     */
    private void changeState(AutoState nextState) {

        if(currentState == AutoState.DRIVE_TO_PARK_Path && nextState!=AutoState.FINISHED){
            return;//do not interrupt parking to earn points
        }
        if (currentState == AutoState.EMERGENCY_PARK_Path && nextState!=AutoState.FINISHED){
            return;//do not interrupt emergency parking
        }
        stopCollection();
        stopShooting();

        previousState = currentState;
        lastTransition = previousState +"->"+nextState;

        RobotLog.ii("RedAudienceAuto","Time: %.2f Tansition: %s",
                autoTimer.seconds(),lastTransition);
        currentState = nextState;
        stateTimer.reset();

        switch(nextState){
            case INITIAL_APPROACH_Path:
                follower.follow(paths.initialApproach);
                break;
            case INITIAL_SHOOT:
            case SHOOT_FLOWER_POLLEN:
            case SHOOT_GARDEN_POLLEN:
                beginShooting();
                break;
            case DRIVE_TO_GARDEN_Path:
                follower.follow(paths.driveToGarden);
                break;
            case COLLECT_FLOWER:
                beginCollectionFlower();
                break;
            case COLLECT_GARDEN:
                beginCollectionGarden();
                break;
            case GARDEN_TO_AUDIENCE_Path:
                follower.follow(paths.gardenToAudience);
                break;
            case DRIVE_TO_FLOWER_Path:
                follower.follow(paths.driveToFlower);
                break;
            case FLOWER_TO_AUDIENCE_Path:
                follower.follow(paths.flowerToAudience);
                break;
            case EMERGENCY_PARK_Path:
                follower.stop();
                Path parkingPath = paths.emergencyPark(follower.pose());
                parkingPathStarted = true;
                follower.follow(parkingPath);
                break;
            case DRIVE_TO_PARK_Path:

                parkingPathStarted = true;
                follower.follow(paths.driveToPark);
                break;
            case WAIT_FOR_AUDIENCE_SCORABLE:
                break;
            //case WAIT_FOR_AUDIENCE_AGAIN:
            case CONFIRM_AUDIENCE_SCORABLE:
                break;
            case FINISHED:
                finished = true;
                break;

        }
    }

    /**
     * Determines whether the robot must begin parking.
     *
     * @return true when the parking reserve has been reached
     */
    private boolean mustParkNow() {
        return autoTimer.seconds()
                >= AUTO_DURATION_SECONDS - PARK_RESERVE_SECONDS;
    }


    /**
     * Checks whether the active Pedro path has completed.
     *
     * @return true if the follower is no longer following a path
     */
    private boolean pathFinished() {
        return !follower.isBusy();
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
         *
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
     * Determines whether the Red Audience CELL is scorable.
     *
     * @return true when Vision confirms a scorable CELL
     */
    private boolean isAudienceCellScorable() {

        if (NAVIGATION_ONLY) {
            return stateTimer.seconds()>= AUDIENCE_WAIT;
        }

        /*
         * TODO: Call your existing Vision roll-based
         * scorable CELL detection method.
         */
        return false;
    }

    /**
     * Checks whether the next task can be completed while
     * preserving enough time for parking.
     */
    private boolean hasTimeFor(double taskSeconds, double parkSeconds){
        double remaining = AUTO_DURATION_SECONDS - autoTimer.seconds();
        return remaining >= taskSeconds + parkSeconds+SAFETY_MARGIN_SECONDS;
    }

    @Override
    public void stop() {
        stopCollection();
        stopShooting();

        if (follower != null) {
            follower.stop();
        }
    }
}

