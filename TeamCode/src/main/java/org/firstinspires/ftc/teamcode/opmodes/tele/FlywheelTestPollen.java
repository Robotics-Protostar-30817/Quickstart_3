
package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

import java.util.Locale;

/**
 * Measures flywheel RPM drop and recovery after a POLLEN shot.
 *
 * Hardware:
 * - One goBILDA Yellow Jacket 6000 RPM motor
 * - Motor encoder
 * - 1:1 belt drive
 *
 * Controls:
 * A = Start flywheel
 * B = Stop flywheel
 * D-pad Up/Down = Adjust target RPM by 100
 * X = Mark beginning of shot
 * Y = Clear previous result
 *
 * IMPORTANT:
 * This OpMode does NOT control the feeder.
 * Use a separate, guarded feeding mechanism.
 * Never feed POLLEN by hand into a spinning flywheel.
 */
@TeleOp(name = "Flywheel Test POLLEN", group = "Test")
public final class FlywheelTestPollen extends OpMode {

    private static final String MOTOR_NAME = "flywheel";

    private static final double INITIAL_RPM = 3000.0;
    private static final double RPM_STEP = 100.0;
    private static final double MIN_TARGET_RPM = 500.0;
    private static final double MAX_TEST_RPM = 5000.0;

    // Time window for detecting the RPM drop.
    private static final double DIP_WINDOW_SECONDS = 0.75;

    // Recovery means reaching 95% of pre-shot RPM.
    private static final double RECOVERY_FRACTION = 0.95;

    // Require recovery speed to remain stable for 150 ms.
    private static final double RECOVERY_HOLD_SECONDS = 0.15;

    private static final double TIMEOUT_SECONDS = 5.0;

    private enum MeasurementState {
        IDLE,
        CAPTURING,
        WAITING_FOR_RECOVERY,
        COMPLETE,
        TIMEOUT
    }

    private final Flywheel flywheel = Flywheel.INSTANCE;
    private final ElapsedTime clock = new ElapsedTime();

    private double selectedRPM = INITIAL_RPM;
    private boolean running = false;

    private boolean previousA = false;
    private boolean previousB = false;
    private boolean previousX = false;
    private boolean previousY = false;
    private boolean previousUp = false;
    private boolean previousDown = false;

    private MeasurementState state = MeasurementState.IDLE;

    private double shotStart = 0.0;
    private double preShotRPM = 0.0;
    private double minRPM = 0.0;
    private double minTime = 0.0;

    private double recoveryThreshold = 0.0;
    private double aboveThresholdSince = -1.0;
    private double recoverySeconds = Double.NaN;

    private int completedShots = 0;

    @Override
    public void init() {

        DcMotorEx motor =
                hardwareMap.get(DcMotorEx.class, MOTOR_NAME);

        flywheel.init(motor);
        flywheel.stop();

        running = false;
        clock.reset();

        telemetry.addLine("Flywheel Test POLLEN");
        telemetry.addLine("A: START");
        telemetry.addLine("B: STOP");
        telemetry.addLine("X: Mark shot");
        telemetry.addLine("Y: Clear result");
        telemetry.addLine("D-pad Up/Down: +/-100 RPM");

        telemetry.addData(
                "Encoder ticks/rev",
                "%.2f",
                flywheel.getTicksPerRevolution()
        );

        telemetry.update();
    }

    @Override
    public void loop() {

        double now = clock.seconds();

        // Increase target RPM.
        if (gamepad1.dpad_up && !previousUp) {

            selectedRPM = Math.min(
                    MAX_TEST_RPM,
                    selectedRPM + RPM_STEP
            );

            if (running) {
                flywheel.setFlywheelRPM(selectedRPM);
            }
        }

        // Decrease target RPM.
        if (gamepad1.dpad_down && !previousDown) {

            selectedRPM = Math.max(
                    MIN_TARGET_RPM,
                    selectedRPM - RPM_STEP
            );

            if (running) {
                flywheel.setFlywheelRPM(selectedRPM);
            }
        }

        // Start flywheel.
        if (gamepad1.a && !previousA) {
            running = true;
            flywheel.setFlywheelRPM(selectedRPM);
        }

        // Stop flywheel.
        if (gamepad1.b && !previousB) {
            running = false;
            flywheel.stop();

            if (isMeasuring()) {
                state = MeasurementState.IDLE;
            }
        }

        // Clear previous measurement.
        if (gamepad1.y && !previousY && !isMeasuring()) {
            state = MeasurementState.IDLE;
            recoverySeconds = Double.NaN;
            preShotRPM = 0.0;
            minRPM = 0.0;
        }

        // Read actual flywheel speed.
        double actualRPM =
                Math.abs(flywheel.getFlywheelRPM());

        // Mark shot start when X is pressed.
        if (gamepad1.x
                && !previousX
                && running
                && !isMeasuring()
                && actualRPM >= selectedRPM * 0.90) {

            state = MeasurementState.CAPTURING;

            shotStart = now;
            preShotRPM = actualRPM;

            minRPM = actualRPM;
            minTime = now;

            recoveryThreshold =
                    preShotRPM * RECOVERY_FRACTION;

            aboveThresholdSince = -1.0;
            recoverySeconds = Double.NaN;
        }

        // Analyze RPM during and after the shot.
        if (isMeasuring()) {

            // Capture the minimum RPM.
            if (actualRPM < minRPM) {
                minRPM = actualRPM;
                minTime = now;
                aboveThresholdSince = -1.0;
            }

            // Measurement timeout.
            if (now - shotStart >= TIMEOUT_SECONDS) {

                state = MeasurementState.TIMEOUT;

            } else {

                // Check recovery threshold.
                if (actualRPM >= recoveryThreshold
                        && now > minTime) {

                    if (aboveThresholdSince < 0.0) {
                        aboveThresholdSince = now;
                    }

                } else {
                    aboveThresholdSince = -1.0;
                }

                // End minimum-RPM capture window.
                if (state == MeasurementState.CAPTURING
                        && now - shotStart >= DIP_WINDOW_SECONDS) {

                    state =
                            MeasurementState.WAITING_FOR_RECOVERY;
                }

                // Confirm stable recovery.
                if (state == MeasurementState.WAITING_FOR_RECOVERY
                        && minRPM < recoveryThreshold
                        && aboveThresholdSince >= 0.0
                        && now - aboveThresholdSince
                        >= RECOVERY_HOLD_SECONDS) {

                    recoverySeconds = now - minTime;

                    state = MeasurementState.COMPLETE;
                    completedShots++;
                }
            }
        }

        // Store button states.
        previousA = gamepad1.a;
        previousB = gamepad1.b;
        previousX = gamepad1.x;
        previousY = gamepad1.y;
        previousUp = gamepad1.dpad_up;
        previousDown = gamepad1.dpad_down;

        flywheel.periodic();

        // Driver Station telemetry.
        telemetry.addLine(
                "=== FLYWHEEL POLLEN SHOT TEST ==="
        );

        telemetry.addData(
                "Motor",
                running ? "RUNNING" : "STOPPED"
        );

        telemetry.addData(
                "Target RPM",
                "%.0f",
                selectedRPM
        );

        telemetry.addData(
                "Current Flywheel RPM",
                "%.0f",
                actualRPM
        );

        telemetry.addData(
                "Encoder Ticks/Rev",
                "%.2f",
                flywheel.getTicksPerRevolution()
        );

        telemetry.addData("Measurement", state);
        telemetry.addData("Completed Shots", completedShots);

        if (state != MeasurementState.IDLE) {

            double rpmDrop = Math.max(
                    0.0,
                    preShotRPM - minRPM
            );

            double dropPercent =
                    preShotRPM > 0.0
                            ? 100.0 * rpmDrop / preShotRPM
                            : 0.0;

            telemetry.addData(
                    "Pre-shot RPM",
                    "%.0f",
                    preShotRPM
            );

            telemetry.addData(
                    "Minimum RPM",
                    "%.0f",
                    minRPM
            );

            telemetry.addData(
                    "RPM Drop",
                    "%.0f (%.1f%%)",
                    rpmDrop,
                    dropPercent
            );

            telemetry.addData(
                    "95%% Recovery Threshold",
                    "%.0f",
                    recoveryThreshold
            );

            telemetry.addData(
                    "Recovery Time (s)",
                    Double.isNaN(recoverySeconds)
                            ? "Pending / not detected"
                            : String.format(
                            Locale.US,
                            "%.3f",
                            recoverySeconds
                    )
            );
        }

        telemetry.addLine("");
        telemetry.addLine("A: START   B: STOP");
        telemetry.addLine("X: Mark shot   Y: Clear");
        telemetry.addLine("D-pad Up/Down: +/-100 RPM");
        telemetry.update();
    }

    private boolean isMeasuring() {
        return state == MeasurementState.CAPTURING
                || state == MeasurementState.WAITING_FOR_RECOVERY;
    }

    @Override
    public void stop() {
        running = false;
        flywheel.stop();
    }
}
