package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Collect successful shot settings for a stationary robot.
 * Does NOT fire the feeder or save files. Copy the logged rows into AutoAim.SHOT_TABLE.
 * Verify camera coordinate conversion, turret gearing/direction and mechanical limits before testing.
 * RB enables automatic turret aiming; LB disables it. Auto aim starts disabled.
 */
@TeleOp(name = "Shot Calibration", group = "Test")
public class ShotCalibration extends OpMode {
    private final Vision vision = Vision.INSTANCE;
    private final Hood hood = Hood.INSTANCE;
    private final Flywheel flywheel = Flywheel.INSTANCE;
    private final Turret turret = Turret.INSTANCE;

    // Set to -1.0 if a target to robot-right requires negative turret angles.
    // Verify with low-power testing before enabling automatic aiming.
    public static double TURRET_DIRECTION = 1.0;

    private boolean autoTurretEnabled = false;
    private boolean prevRb, prevLb;
    private Vision.Cell.Color color = Vision.Cell.Color.RED;
    private double requestedRpm = 2500.0; // starting test value only
    private boolean prevUp, prevDown, prevLeft, prevRight, prevA, prevX, prevY;
    private final List<String> acceptedRows = new ArrayList<>();
    private String status = "Ready";

    @Override public void init() {
        vision.init(hardwareMap, telemetry);
        hood.init(hardwareMap.get(Servo.class, "hood"));
        flywheel.init(hardwareMap.get(DcMotorEx.class, "flywheel"));
        turret.init(hardwareMap.get(DcMotorEx.class, "turret"));
        flywheel.stop();
        turret.stop();
        autoTurretEnabled = false;
        telemetry.addLine("Center turret forward before INIT. Confirm Vision XYZ first.");
        telemetry.addLine("NO automatic feeder actuation. Shoot manually using separate safe control.");
        telemetry.update();
    }

    @Override public void loop() {
        // Change these values only after confirming safe hardware limits.
        if (gamepad1.dpad_up && !prevUp) hood.adjustPosition(+0.005);
        if (gamepad1.dpad_down && !prevDown) hood.adjustPosition(-0.005);
        if (gamepad1.dpad_right && !prevRight) requestedRpm = Math.min(5000, requestedRpm + 100);
        if (gamepad1.dpad_left && !prevLeft) requestedRpm = Math.max(500, requestedRpm - 100);
        if (gamepad1.x && !prevX) color = Vision.Cell.Color.RED;
        if (gamepad1.y && !prevY) color = Vision.Cell.Color.BLUE;

        // Right bumper enables vision aiming; left bumper immediately disables it.
        // Disable wins if both bumpers are pressed together.
        if (gamepad1.right_bumper && !prevRb) autoTurretEnabled = true;
        if (gamepad1.left_bumper && !prevLb) autoTurretEnabled = false;

        // Hold B to spin flywheel; releasing stops motor power (wheel can coast).
        if (gamepad1.b) flywheel.setFlywheelRPM(requestedRpm);
        else flywheel.stop();
        flywheel.periodic();

        Vision.Cell cell = vision.getBestCell(color);
        double distance = Double.NaN;
        double direction = Double.NaN;
        boolean valid = cell != null && cell.isScorable()
                && Double.isFinite(cell.getX()) && Double.isFinite(cell.getY())
                && Double.isFinite(cell.getZ());
        if (valid) {
            distance = Math.hypot(cell.getX(), cell.getY());
            direction = Math.toDegrees(Math.atan2(cell.getX(), cell.getY()));
            valid = distance > 1.0;
        }

        // Robot-centered vision coordinates and robot-centered turret axis:
        // +X right, +Y forward. No translational offset is required.
        double requestedTurretAngle = valid ? TURRET_DIRECTION * direction : Double.NaN;
        boolean turretInRange = valid && Double.isFinite(requestedTurretAngle)
                && requestedTurretAngle >= Turret.MIN_ANGLE_DEG
                && requestedTurretAngle <= Turret.MAX_ANGLE_DEG;

        // Never rely on setTargetAngle() clipping an out-of-range target.
        // Stop rather than continuing to pursue a stale target when vision is lost.
        if (autoTurretEnabled && turretInRange) {
            turret.setTargetAngle(requestedTurretAngle);
        } else {
            turret.stop();
        }
        turret.periodic(); // Required to actually drive the turret motor.
        boolean turretReady = autoTurretEnabled && turretInRange && turret.isAtTarget();

        // A records an operator-confirmed SUCCESSFUL shot only. Do not record a miss.
        // Keep the robot stationary and verify the shot landed before pressing A.
        if (gamepad1.a && !prevA) {
            if (!valid) status = "Not recorded: no valid scorable CELL";
            else if (flywheel.isVelocityMode()) status = "Release B after shot, then press A to record";
            else {
                String row = String.format(Locale.US, "{%.2f, %.4f, %.0f},",
                        distance, hood.getPosition(), requestedRpm);
                acceptedRows.add(row);
                status = "Recorded: " + row;
            }
        }

        telemetry.addData("Alliance", color);
        telemetry.addData("Scorable CELL", valid ? cell.getName() : "NONE");
        if (valid) {
            telemetry.addData("XYZ (in)", "%.1f, %.1f, %.1f", cell.getX(), cell.getY(), cell.getZ());
            telemetry.addData("Horizontal distance (in)", "%.2f", distance);
            telemetry.addData("Turret direction (deg)", "%.1f", direction);
        }
        telemetry.addData("Auto turret", autoTurretEnabled ? "ENABLED" : "DISABLED");
        telemetry.addData("Turret current (deg)", "%.1f", turret.getAngleDegrees());
        telemetry.addData("Turret requested (deg)", turretInRange ? String.format(Locale.US, "%.1f", requestedTurretAngle) : "UNAVAILABLE / OUT OF RANGE");
        telemetry.addData("Turret limits (deg)", "%.1f to %.1f", Turret.MIN_ANGLE_DEG, Turret.MAX_ANGLE_DEG);
        telemetry.addData("Turret aligned", turretReady);
        telemetry.addData("Hood command", "%.4f", hood.getPosition());
        telemetry.addData("Requested RPM", "%.0f", requestedRpm);
        telemetry.addData("Measured RPM", "%.0f", flywheel.getFlywheelRPM());
        telemetry.addData("Flywheel running", gamepad1.b);
        telemetry.addData("Accepted rows", acceptedRows.size());
        telemetry.addData("Status", status);
        telemetry.addLine("Dpad up/down hood +/-0.005; left/right RPM +/-100");
        telemetry.addLine("RB auto turret ON | LB auto turret OFF");
        telemetry.addLine("Only fire when turret aligned; verify aiming direction first.");
        telemetry.addLine("Hold B spin; X RED / Y BLUE; A log successful shot AFTER release");
        telemetry.addLine("COPY THESE ROWS into AutoAim.SHOT_TABLE (sort by distance):");
        for (String row : acceptedRows) telemetry.addLine(row);
        telemetry.update();

        prevUp = gamepad1.dpad_up; prevDown = gamepad1.dpad_down;
        prevLeft = gamepad1.dpad_left; prevRight = gamepad1.dpad_right;
        prevA = gamepad1.a; prevX = gamepad1.x; prevY = gamepad1.y;
        prevRb = gamepad1.right_bumper; prevLb = gamepad1.left_bumper;
    }

    @Override public void stop() {
        flywheel.stop();
        turret.stop();
        vision.close();
    }
}

