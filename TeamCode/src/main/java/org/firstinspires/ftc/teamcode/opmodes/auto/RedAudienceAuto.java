
package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

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
@Autonomous(name = "Red Audience Auto", group = "Autonomous")
public class RedAudienceAuto extends OpMode {

    /**
     * States used exclusively by this autonomous routine.
     */
    private enum AutoState {
        INITIAL_SHOOT,
        SHOOT,
        DRIVE_TO_GARDEN,
        COLLECT_GARDEN,

        GARDEN_TO_AUDIENCE,
        WAIT_FOR_AUDIENCE,
        SHOOT_GARDEN_POLLEN,

        DRIVE_TO_FLOWER,
        COLLECT_FLOWER,

        Flower_TO_AUDIENCE,
        //WAIT_FOR_AUDIENCE_AGAIN,
        SHOOT_FLOWER_POLLEN,

        DRIVE_TO_PARK,
        FINISHED
    }

    /**
     * Set true for testing navigation without activating the
     * intake or shooting mechanism.
     */
    private static final boolean NAVIGATION_ONLY = true;

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
    private static final double TEST_COLLECTION_SECONDS = 2.0;

    /**
     * Temporary shooting duration for navigation-only testing.
     */
    private static final double TEST_SHOOT_SECONDS = 2.0;

    private Follower follower;

    private AutoState currentState;


    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();

    private boolean stateJustEntered;

    /**
     * Robot poses in Pedro's field coordinate system.
     *
     * IMPORTANT: These are placeholders, not measured field poses.
     * Replace them before running the robot.
     */
    private Pose startPose;
    private Pose gardenPose;
    private Pose shootPose;
    private Pose flowerPose;
    private Pose parkPose;

    @Override
    public void init() {

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
        initializePoses();

        if (follower != null) {
            follower.setPose(startPose);
        }

        currentState = AutoState.INITIAL_SHOOT;
        stateJustEntered = true;

        telemetry.addLine("Red Audience Auto initialized");
        telemetry.addData("Navigation only", NAVIGATION_ONLY);
        telemetry.addLine("Configure follower and field poses before running.");
        telemetry.update();
        //autoTimer = new ElapsedTime();
        //stateTimer = new ElapsedTime();
    }

    /**
     * Defines the starting, shooting, collection, and parking poses.
     *
     * <p>All coordinates are placeholders until measured using
     * your Pedro field coordinate system.
     */
    private void initializePoses() {

        startPose = new Pose(60, 10, 90);
        shootPose = new Pose(60,30,90);
        gardenPose = new Pose(12, 14, -90);

        //audiencePose = new Pose(0, 0, 0);
        flowerPose = new Pose(13.5, 48, 180);
        parkPose = new Pose(12, 97, 135);
    }

    @Override
    public void start() {
        autoTimer.reset();
        stateTimer.reset();
    }

    @Override
    public void loop() {

        if (follower == null) {
            telemetry.addLine("Follower not initialized");
            telemetry.update();
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

            case INITIAL_SHOOT:

                if (onStateEntry()) {
                    driveTo(shootPose);
                    beginShooting();
                }

                if (shootingFinished()) {
                    changeState(AutoState.DRIVE_TO_GARDEN);
                }
                break;

            case DRIVE_TO_GARDEN:

                if (onStateEntry()) {
                    driveTo(gardenPose);
                }

                if (pathFinished()) {
                    changeState(AutoState.COLLECT_GARDEN);
                }
                break;

            case COLLECT_GARDEN:

                if (onStateEntry()) {
                    beginCollectionGarden();
                }

                if (collectionFinished()) {
                    stopCollectionGarden();
                    changeState(AutoState.GARDEN_TO_AUDIENCE);
                }
                break;

            case GARDEN_TO_AUDIENCE:

                if (onStateEntry()) {
                    driveTo(shootPose);
                }

                if (pathFinished()) {
                    changeState(AutoState.WAIT_FOR_AUDIENCE);
                }
                break;

            case WAIT_FOR_AUDIENCE:

                if (isAudienceCellScorable()) {
                    changeState(AutoState.SHOOT_GARDEN_POLLEN);
                }
                break;

            case SHOOT_GARDEN_POLLEN:

                if (onStateEntry()) {
                    beginShooting();
                }

                if (shootingFinished()) {
                    if (canAttemptFlowerCycle()) {
                        changeState(AutoState.DRIVE_TO_FLOWER);
                    }
                }
                break;

            case DRIVE_TO_FLOWER:

                if (onStateEntry()) {
                    driveTo(flowerPose);
                }

                if (pathFinished()) {
                    changeState(AutoState.COLLECT_FLOWER);
                }
                break;

            case COLLECT_FLOWER:

                if (onStateEntry()) {
                    beginCollection();
                }

                if (collectionFinished()) {
                    stopFlowerCollection();
                    changeState(AutoState.Flower_TO_AUDIENCE);
                }
                break;

            case Flower_TO_AUDIENCE:

                if (onStateEntry()) {
                    driveTo(shootPose);
                }

                if (pathFinished()&& isAudienceCellScorable()) {
                    changeState(AutoState.SHOOT_FLOWER_POLLEN);
                }
                break;

            case SHOOT_FLOWER_POLLEN:

                if (onStateEntry()) {
                    beginShooting();
                }

                if (shootingFinished()) {
                    changeState(AutoState.DRIVE_TO_PARK);
                }
                break;

            case DRIVE_TO_PARK:

                if (onStateEntry()) {
                    stopCollection();
                    stopShooting();
                    driveTo(parkPose);
                }

                if (pathFinished()) {
                    changeState(AutoState.FINISHED);
                }
                break;

            case FINISHED:

                if (onStateEntry()) {
                    stopCollection();
                    stopShooting();
                }
                break;
        }

        telemetry.addData("State", currentState);
        telemetry.addData("Auto time", "%.1f", autoTimer.seconds());
        telemetry.addData("Remaining", "%.1f",
                Math.max(0, AUTO_DURATION_SECONDS - autoTimer.seconds()));
        telemetry.addData("Path busy", follower.isBusy());
        telemetry.update();
    }

    /**
     * Changes the autonomous state and resets its elapsed time.
     *
     * @param nextState state to execute on the next loop
     */
    private void changeState(AutoState nextState) {
        currentState = nextState;
        stateTimer.reset();
        stateJustEntered = true;
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
    private boolean canAttemptFlowerCycle() {

        /*
         * TODO: Replace this placeholder with measured travel,
         * collection, shooting, and parking durations.
         *
         * Until then, disable the optional cycle in live operation.
         */
        return NAVIGATION_ONLY
                && autoTimer.seconds() < 15.0;
    }

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
            return stateTimer.seconds() >= TEST_SHOOT_SECONDS;
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

        /*
         * TODO: Stop the feeder and flywheel safely.
         */
    }

    /**
     * Starts the POLLEN intake.
     */
    private void beginCollection() {

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
            return stateTimer.seconds() >= TEST_COLLECTION_SECONDS;
        }

        /*
         * TODO: Return true when the robot is full
         * (maximum four POLLEN) or the collection timeout expires.
         */
        return false;
    }

    /**
     * Stops the POLLEN intake.
     */
    private void stopCollection() {

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
            return true;
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
            follower.breakFollowing();
        }
    }
}

