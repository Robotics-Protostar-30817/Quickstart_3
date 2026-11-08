
package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

/**
 * One-button continuous-feed test for up to FOUR loaded POLLEN.
 *
 * A: Spin up flywheel
 * X: Start one timed feeder run
 * B: Stop BOTH motors
 * Y: Clear results
 * D-pad up/down: Adjust flywheel RPM
 *
 * IMPORTANT:
 * Without a POLLEN exit sensor, the software cannot count
 * actual shots or determine when the magazine is empty.
 *
 * FEED_SECONDS must be calibrated for the actual feeder.
 *
 * Measures minimum flywheel RPM and recovery time for the
 * entire feeding sequence, not individual POLLEN.
 */
@TeleOp(name = "Flywheel 4 POLLEN Test", group = "Test")
public final class FlywheelTestMultiplePollen extends OpMode {

    private static final String FLYWHEEL_NAME = "flywheel";
    private static final String FEEDER_NAME = "feeder";

    private static final int MAX_LOADED_POLLEN = 4;

    private static final double INITIAL_RPM = 3000.0;
    private static final double MIN_RPM = 500.0;
    private static final double MAX_TEST_RPM = 5000.0;
    private static final double RPM_STEP = 100.0;

    private static final double START_TOLERANCE = 0.95;
    private static final double STABLE_BEFORE_FEED_S = 0.20;

    // Calibrate these values for the actual feeder.
    // 0.50 seconds is NOT a verified duration for four POLLEN.
    private static final double FEEDER_POWER = 0.35;
    private static final double FEED_SECONDS = 0.50;
    private static final double MAX_FEED_SECONDS = 3.0;

    private static final double RECOVERY_FRACTION = 0.95;
    private static final double RECOVERY_HOLD_S = 0.15;
    private static final double RECOVERY_TIMEOUT_S = 5.0;
    private static final double MIN_DIP_FRACTION = 0.01;

    private enum State {
        IDLE,
        FEEDING,
        RECOVERING,
        COMPLETE,
        TIMEOUT,
        CANCELLED
    }

    private final Flywheel flywheel = Flywheel.INSTANCE;
    private final Feeder feeder = Feeder.INSTANCE;
    private final ElapsedTime timer = new ElapsedTime();

    private State state = State.IDLE;

    private boolean running;

    private boolean prevA, prevB, prevX, prevY;
    private boolean prevUp, prevDown;

    private double selectedRPM = INITIAL_RPM;
    private double stableSince = -1.0;

    private double feedStart;
    private double feedEnd;
    private double recoveryStableSince = -1.0;

    private double preFeedRPM;
    private double minRPM;
    private double minTime;
    private double endFeedRPM;

    private double recoveryTime = Double.NaN;
    private double feedDuration = Double.NaN;

    private int completedSequences;

    @Override
    public void init() {

        flywheel.init(
                hardwareMap.get(DcMotorEx.class, FLYWHEEL_NAME)
        );

        feeder.init(
                hardwareMap.get(DcMotorEx.class, FEEDER_NAME)
        );

        flywheel.stop();
        feeder.stop();

        running = false;
        state = State.IDLE;

        timer.reset();

        telemetry.addLine("A spin up | X feed sequence | B stop both");
        telemetry.addLine("Y clear | D-pad +/-100 RPM");
        telemetry.addLine("CALIBRATE FEED_SECONDS; NO POLLEN COUNT SENSOR");

        telemetry.addData(
                "Encoder ticks/rev",
                "%.2f",
                flywheel.getTicksPerRevolution()
        );

        telemetry.update();
    }

    private boolean busy() {
        return state == State.FEEDING
                || state == State.RECOVERING;
    }

    private void cancel() {

        feeder.stop();
        flywheel.stop();

        running = false;
        stableSince = -1.0;

        if (busy()) {
            state = State.CANCELLED;
        }
    }

    @Override
    public void loop() {

        double now = timer.seconds();

        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;
        boolean y = gamepad1.y;

        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;

        // B always has priority.
        if (b) {
            cancel();

        } else {

            if (!busy()) {

                if (up && !prevUp) {
                    selectedRPM = Math.min(
                            MAX_TEST_RPM,
                            selectedRPM + RPM_STEP
                    );
                }

                if (down && !prevDown) {
                    selectedRPM = Math.max(
                            MIN_RPM,
                            selectedRPM - RPM_STEP
                    );
                }

                if (running &&
                        ((up && !prevUp) || (down && !prevDown))) {

                    flywheel.setFlywheelRPM(selectedRPM);
                    stableSince = -1.0;
                }

                if (a && !prevA && !running) {

                    flywheel.setFlywheelRPM(selectedRPM);
                    running = true;
                    stableSince = -1.0;
                }

                if (y && !prevY) {

                    state = State.IDLE;
                    completedSequences = 0;

                    recoveryTime = Double.NaN;
                    feedDuration = Double.NaN;
                }
            }

            double rpm = Math.abs(flywheel.getFlywheelRPM());

            boolean nearTarget =
                    running
                            && rpm >= selectedRPM * START_TOLERANCE
                            && rpm <= selectedRPM * 1.05;

            // Wait until flywheel speed is stable.
            if (!busy()) {

                if (nearTarget) {

                    if (stableSince < 0.0) {
                        stableSince = now;
                    }

                } else {
                    stableSince = -1.0;
                }

                // X starts the entire feeding sequence.
                if (x && !prevX
                        && stableSince >= 0.0
                        && now - stableSince >= STABLE_BEFORE_FEED_S) {

                    preFeedRPM = rpm;
                    minRPM = rpm;
                    minTime = now;

                    feedStart = now;

                    recoveryStableSince = -1.0;
                    recoveryTime = Double.NaN;
                    feedDuration = Double.NaN;

                    feeder.start(FEEDER_POWER);

                    state = State.FEEDING;
                }
            }

            // Track the lowest RPM during the sequence.
            if (state == State.FEEDING
                    || state == State.RECOVERING) {

                if (rpm < minRPM) {

                    minRPM = rpm;
                    minTime = now;
                    recoveryStableSince = -1.0;
                }
            }

            // Automatically stop the feeder.
            if (state == State.FEEDING
                    && now - feedStart >=
                    Math.min(FEED_SECONDS, MAX_FEED_SECONDS)) {

                feeder.stop();

                feedEnd = now;
                feedDuration = feedEnd - feedStart;
                endFeedRPM = rpm;

                state = State.RECOVERING;
            }

            // Measure flywheel recovery after feeding.
            if (state == State.RECOVERING) {

                double threshold =
                        preFeedRPM * RECOVERY_FRACTION;

                boolean meaningfulDip =
                        minRPM <= preFeedRPM
                                * (1.0 - MIN_DIP_FRACTION);

                if (meaningfulDip && rpm >= threshold) {

                    if (recoveryStableSince < 0.0) {
                        recoveryStableSince = now;
                    }

                    if (now - recoveryStableSince
                            >= RECOVERY_HOLD_S) {

                        recoveryTime = now - feedEnd;

                        completedSequences++;
                        state = State.COMPLETE;
                    }

                } else {
                    recoveryStableSince = -1.0;
                }

                if (state == State.RECOVERING
                        && now - feedEnd >= RECOVERY_TIMEOUT_S) {

                    state = State.TIMEOUT;
                }
            }
        }

        flywheel.periodic();
        feeder.periodic();

        double rpm = Math.abs(flywheel.getFlywheelRPM());

        telemetry.addLine("=== 4-POLLEN FEED SEQUENCE TEST ===");

        telemetry.addData(
                "Flywheel",
                running ? "RUNNING" : "STOPPED"
        );

        telemetry.addData(
                "Target / actual RPM",
                "%.0f / %.0f",
                selectedRPM,
                rpm
        );

        telemetry.addData(
                "Feeder",
                feeder.isRunning() ? "RUNNING" : "STOPPED"
        );

        telemetry.addData("State", state);
        telemetry.addData("Max capacity", MAX_LOADED_POLLEN);

        telemetry.addLine("Actual POLLEN count NOT measured");

        telemetry.addData(
                "Feed setting",
                "%.2f power, %.2f s",
                FEEDER_POWER,
                FEED_SECONDS
        );

        if (state != State.IDLE) {

            double drop = Math.max(
                    0.0,
                    preFeedRPM - minRPM
            );

            telemetry.addData(
                    "Pre-feed / minimum RPM",
                    "%.0f / %.0f",
                    preFeedRPM,
                    minRPM
            );

            telemetry.addData(
                    "Sequence RPM drop",
                    "%.0f (%.1f%%)",
                    drop,
                    preFeedRPM > 0.0
                            ? 100.0 * drop / preFeedRPM
                            : 0.0
            );

            telemetry.addData(
                    "RPM at feeder stop",
                    "%.0f",
                    endFeedRPM
            );

            telemetry.addData(
                    "Actual feed duration",
                    Double.isNaN(feedDuration)
                            ? "Feeding / not completed"
                            : String.format(
                            java.util.Locale.US,
                            "%.3f s",
                            feedDuration
                    )
            );

            telemetry.addData(
                    "Recovery after feed stop",
                    Double.isNaN(recoveryTime)
                            ? "Pending / not detected"
                            : String.format(
                            java.util.Locale.US,
                            "%.3f s",
                            recoveryTime
                    )
            );
        }

        telemetry.addData(
                "Completed sequences",
                completedSequences
        );

        telemetry.addLine(
                "A spin up | X start entire feed | B stop both"
        );

        telemetry.update();

        prevA = a;
        prevB = b;
        prevX = x;
        prevY = y;

        prevUp = up;
        prevDown = down;
    }

    @Override
    public void stop() {
        cancel();
    }
}

