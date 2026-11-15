
package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Hood;

/**
 * Manual test for the shooter hood servo.
 *
 * Hood servo position-to-Physical-launch-angle calibration.
 * Angle values are entered by the operator after measureing the hood angle.
 * Records are telemetry-only, copy them before stopping the OpMode.
 *
 * D-pad Up:   Increase servo position by 0.01
 * D-pad Down: Decrease servo position by 0.01
 * A:          Set position to 0.50
 * X:          Set position to 0.45
 * Y:          Set position to 0.55
 * B:          Block new commands while held
 * Left/Right bumper: Measure angle -1 /+1 degree
 * Left/Right trigger: Measureed angle -0.1/+0.1 degree
 * Start: Record calibration pair
 * BACk: clear recorded measurement
 *
 * Servo hardware configuration name: "hood"
 */
@TeleOp(name = "Hood Manual Test", group = "Test")
public class HoodManualTest extends OpMode {
    private static final String SERVO_NAME = "hood";
    private static final int MAX_SAMPLES = 20;
    private final Hood hood = Hood.INSTANCE;
    private final double[] positions = new double[MAX_SAMPLES];
    private final double[] angles = new double[MAX_SAMPLES];
    private int sampleCount = 0;
    private double measuredAngleDeg = 0.0;
    private boolean angleEntered = false;
    private boolean prevUp, prevDown, prevA, prevX, prevY;
    private boolean prevLeftBumper, prevRightBumper, prevLeftTrigger, prevRightTrigger;
    private boolean prevStart, prevBack;
    private String message = "Measure physical launch angle before recording";

    @Override public void init() {
        hood.init(hardwareMap.get(Servo.class, SERVO_NAME));
        telemetry.addLine("Remove POLLEN. Verify safe hood limits before motion.");
        telemetry.addLine("D-pad up/down: servo +/- Hood.TEST_STEP");
        telemetry.addLine("A/X/Y: servo 0.50/0.45/0.55; hold B to block servo moves");
        telemetry.addLine("LB/RB: measured angle -/+ 1 deg");
        telemetry.addLine("LT/RT (press): measured angle -/+ 0.1 deg");
        telemetry.addLine("START: record measured angle and servo position");
        telemetry.addLine("BACK: clear recorded samples");
        telemetry.update();
    }

    @Override public void loop() {
        boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;
        boolean a = gamepad1.a, x = gamepad1.x, y = gamepad1.y;
        boolean lb = gamepad1.left_bumper, rb = gamepad1.right_bumper;
        boolean lt = gamepad1.left_trigger > 0.5, rt = gamepad1.right_trigger > 0.5;
        boolean start = gamepad1.start, back = gamepad1.back;

        if (!gamepad1.b) {
            if (up && !prevUp) { hood.adjustPosition(Hood.TEST_STEP); angleEntered = false; }
            if (down && !prevDown) { hood.adjustPosition(-Hood.TEST_STEP); angleEntered = false; }
            if (a && !prevA) { hood.setPosition(0.50); angleEntered = false; }
            if (x && !prevX) { hood.setPosition(0.45); angleEntered = false; }
            if (y && !prevY) { hood.setPosition(0.55); angleEntered = false; }
        }
        if (lb && !prevLeftBumper) { measuredAngleDeg -= 1.0; angleEntered = true; }
        if (rb && !prevRightBumper) { measuredAngleDeg += 1.0; angleEntered = true; }
        if (lt && !prevLeftTrigger) { measuredAngleDeg -= 0.1; angleEntered = true; }
        if (rt && !prevRightTrigger) { measuredAngleDeg += 0.1; angleEntered = true; }
        measuredAngleDeg = Math.round(measuredAngleDeg * 10.0) / 10.0;

        if (start && !prevStart) {
            if (!hood.hasCommand()) message = "Command a servo position first";
            else if (!angleEntered) message = "Enter measured angle AFTER last servo movement";
            else if (sampleCount >= MAX_SAMPLES) message = "Sample table full";
            else {
                positions[sampleCount] = hood.getPosition();
                angles[sampleCount] = measuredAngleDeg;
                sampleCount++;
                message = "Recorded sample " + sampleCount;
            }
        }
        if (back && !prevBack) { sampleCount = 0; message = "Samples cleared"; }

        hood.periodic();
        telemetry.addLine("=== HOOD POSITION / ANGLE CALIBRATION ===");
        telemetry.addData("Servo command (NOT feedback)", "%.3f", hood.getPosition());
        telemetry.addData("Allowed servo range", "%.3f to %.3f", Hood.MIN_POSITION, Hood.MAX_POSITION);
        telemetry.addData("Measured angle ENTERED by operator", "%.1f deg", measuredAngleDeg);
        telemetry.addData("Angle entered since last servo move", angleEntered);
        telemetry.addData("Hood time-ready estimate", hood.isReady());
        telemetry.addData("Samples", "%d/%d", sampleCount, MAX_SAMPLES);
        telemetry.addData("Status", message);
        telemetry.addLine("Position -> measured launch angle (copy before STOP):");
        for (int i = 0; i < sampleCount; i++)
            telemetry.addData("Row " + (i + 1), "%.3f -> %.1f deg", positions[i], angles[i]);
        telemetry.addLine("LB/RB angle +/-1; LT/RT +/-0.1; START record; BACK clear");
        telemetry.addLine("D-pad servo +/-step; A/X/Y presets; hold B blocks servo");
        telemetry.update();

        prevUp = up; prevDown = down; prevA = a; prevX = x; prevY = y;
        prevLeftBumper = lb; prevRightBumper = rb;
        prevLeftTrigger = lt; prevRightTrigger = rt;
        prevStart = start; prevBack = back;
    }

    @Override public void stop() { hood.hold(); }
}
