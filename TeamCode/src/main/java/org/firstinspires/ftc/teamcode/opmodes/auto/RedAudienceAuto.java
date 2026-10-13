
package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.auto.RedAudiencePaths;

import kotlin.sequences.USequencesKt;

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
        INITIAL_APPROACH,
        INITIAL_SHOOT,
        DRIVE_TO_GARDEN,
        COLLECT_GARDEN,

        GARDEN_TO_AUDIENCE,
        WAIT_FOR_AUDIENCE,
        SHOOT_GARDEN_POLLEN,

        DRIVE_TO_FLOWER,
        COLLECT_FLOWER,

        FLOWER_TO_AUDIENCE,
        WAIT_FOR_AUDIENCE_AGAIN,
        SHOOT_FLOWER_POLLEN,

        DRIVE_TO_PARK,
        FINISHED
    }

    /**
     * Set true for testing navigation without activating the
     * intake or shooting mechanism.
     */
    private static final boolean NAVIGATION_ONLY = true;
    private static final boolean ENABLE_FLOWER_CYCLE= false;

    private static final double AUTO_DURATION_SECONDS = 30.0;

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

    /**
     * Temporary shooting duration for navigation-only testing.
     */
    private static final double TEST_SHOOT = 2.0;

    private static final double AUDIENCE_WAIT = 2.0;

    private static final double FLOWER_CYCLE=13.0; //FLOWER+PARKING
    private static final double MAX_CELL_WAIT = 3.0;
    private static final double MAX_COLLECTION_TIME = 3.0;
    private Follower follower;

    private AutoState currentState;
    private RedAudiencePaths paths;
    private boolean emergencyParkStarted;
    private boolean parkingPathStarted;
    private boolean finished;
    private boolean started;

    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();

    private boolean stateJustEntered;

    @Override
    public void init() {
        if(!NAVIGATION_ONLY){
            throw new IllegalStateException("Connect shooter, vision, and intake adapters before live auto");
        }
        /*
         * TODO: Initialize follower using your existing, tested
         * Pedro Pathing initialization code.
         *
         * Example structure:
         *
         * follower = Constants.createFollower(hardwareMap);
         *
         * Use your project's actual Constants class and method.
         */
        follower = Constants.create(hardwareMap);
        paths = new RedAudiencePaths();

        if (follower != null) {
            follower.setPose(paths.startPose);
        }

        currentState = AutoState.INITIAL_APPROACH;
        started = false;
        stateJustEntered = true;
        parkingPathStarted = false;

        telemetry.addLine("Red Audience Auto initialized");
        telemetry.addData("Navigation only", NAVIGATION_ONLY);
        telemetry.addLine("Configure follower and field poses before running.");
        telemetry.update();
        //autoTimer = new ElapsedTime();
        //stateTimer = new ElapsedTime();
    }

    @Override
    public void start() {
        autoTimer.reset();
        stateTimer.reset();
        started = true;
        changeState(AutoState.INITIAL_APPROACH);
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

        if (mustParkNow()
                && currentState != AutoState.DRIVE_TO_PARK
                && currentState != AutoState.FINISHED) {

            changeState(AutoState.DRIVE_TO_PARK);
        }


        switch (currentState) {
            case INITIAL_APPROACH:
                if(pathFinished()) changeState(AutoState.INITIAL_SHOOT);
                break;
            case INITIAL_SHOOT:
                if (shootingFinished()) {
                    changeState(AutoState.DRIVE_TO_GARDEN);
                }
                break;

            case DRIVE_TO_GARDEN:
                if (pathFinished()) {
                    changeState(AutoState.COLLECT_GARDEN);
                }
                break;
            case COLLECT_GARDEN:
                if (collectionFinished()) {
                    //stopCollectionGarden();
                    changeState(AutoState.GARDEN_TO_AUDIENCE);
                }
                break;

            case GARDEN_TO_AUDIENCE:
                if (pathFinished()) {
                    changeState(AutoState.WAIT_FOR_AUDIENCE);
                }
                break;

            case WAIT_FOR_AUDIENCE:

                if (isAudienceCellScorable()) {
                    changeState(AutoState.SHOOT_GARDEN_POLLEN);
                }else if (stateTimer.seconds() >= AUDIENCE_WAIT) {
                    changeState(AutoState.DRIVE_TO_PARK);
                }
                break;

            case SHOOT_GARDEN_POLLEN:
                if (shootingFinished()) {
                    if (canAttemptFlowerCycle()) {
                        changeState(AutoState.DRIVE_TO_FLOWER);
                    }else{
                        changeState(AutoState.DRIVE_TO_PARK);
                    }
                }
                break;

            case DRIVE_TO_FLOWER:
                if (pathFinished()) {
                    changeState(AutoState.COLLECT_FLOWER);
                }
                break;

            case COLLECT_FLOWER:
                if (collectionFinished()) {
                    //stopFlowerCollection();
                    changeState(AutoState.FLOWER_TO_AUDIENCE);
                }
                break;

            case FLOWER_TO_AUDIENCE:
                if (pathFinished()&& isAudienceCellScorable()) {
                    changeState(AutoState.SHOOT_FLOWER_POLLEN);
                }else if(stateTimer.seconds()>=MAX_CELL_WAIT){
                    changeState(AutoState.DRIVE_TO_PARK);
                }
                break;

            case SHOOT_FLOWER_POLLEN:
                if (shootingFinished()) {
                    changeState(AutoState.DRIVE_TO_PARK);
                }
                break;

            case DRIVE_TO_PARK:
                if (parkingPathStarted && pathFinished()) {
                    changeState(AutoState.FINISHED);
                }
                break;

            case FINISHED:
                break;
        }

        telemetry.addData("State", currentState);
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

        if(currentState == AutoState.DRIVE_TO_PARK && nextState!=AutoState.FINISHED){
            return;//do not interrupt parking to earn points
        }
        stopCollection();
        stopShooting();

        currentState = nextState;
        stateTimer.reset();

        switch(nextState){
            case INITIAL_APPROACH:
                follower.follow(paths.initialShoot);
                break;
            case INITIAL_SHOOT:
            case SHOOT_FLOWER_POLLEN:
            case SHOOT_GARDEN_POLLEN:
                beginShooting();
                break;
            case DRIVE_TO_GARDEN:
                follower.follow(paths.driveToGarden);
                break;
            case COLLECT_FLOWER:
                beginCollectionFlower();
                break;
            case COLLECT_GARDEN:
                beginCollectionGarden();
                break;
            case GARDEN_TO_AUDIENCE:
                follower.follow(paths.gardenToAudience);
                break;
            case DRIVE_TO_FLOWER:
                follower.follow(paths.driveToFlower);
                break;
            case FLOWER_TO_AUDIENCE:
                follower.follow(paths.flowerToAudience);
                break;
            case DRIVE_TO_PARK:
                follower.stop();
                Path parkingPath = paths.emergencyPark(follower.pose());
                parkingPathStarted = true;
                follower.follow(parkingPath);
                break;
            case WAIT_FOR_AUDIENCE:
            case WAIT_FOR_AUDIENCE_AGAIN:
            case FINISHED:
                 break;

        }
    }

    /**
     * Returns true once when entering a state.
     *
     * @return true on the first execution of the state
     */
    private boolean onStateEntry() {
        if (stateJustEntered) {
            stateJustEntered = false;
            return true;
        }
        return false;
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
     * Determines whether there is sufficient time to attempt
     * the optional Flower collection and shooting cycle.
     *
     * @return true if the optional cycle may be attempted
     */


    /**
     * Starts a path to the supplied field pose.
     *
     * @param destination target Pedro pose
     */
    private void driveTo(Pose destination) {

        /*
         * TODO: Construct and follow a Pedro path from the
         * current pose to destination.
         *
         * Reuse the working path construction pattern from
         * your existing Pedro sample OpMode.
         *
         * Do not reset the follower's starting pose here.
         */
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
        return ENABLE_FLOWER_CYCLE && AUTO_DURATION_SECONDS - autoTimer.seconds()>=FLOWER_CYCLE;

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
            return stateTimer.seconds()>= MAX_CELL_WAIT;
        }

        /*
         * TODO: Call your existing Vision roll-based
         * scorable CELL detection method.
         */
        return false;
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

