package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.AutoAim;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

/**
 * Diagnostic test for the existing AutoAim.java.
 * Does not fire POLLEN or control a feeder.
 * Hardware names: turret, flywheel, hood, Webcam1.
 */
@TeleOp(name = "AutoAim Manual Test", group = "Test")
public class AutoAimManualTest extends OpMode {
    private final Vision vision = Vision.INSTANCE;
    private final Turret turret = Turret.INSTANCE;
    private final Hood hood = Hood.INSTANCE;
    private final Flywheel flywheel = Flywheel.INSTANCE;
    private final AutoAim autoAim = AutoAim.INSTANCE;

    private Vision.Cell.Color alliance = Vision.Cell.Color.RED;
    private boolean previousA, previousB, previousX, previousY;

    @Override
    public void init() {
        turret.init(hardwareMap.get(DcMotorEx.class, "turret"));
        hood.init(hardwareMap.get(Servo.class, "hood"));
        flywheel.init(hardwareMap.get(DcMotorEx.class, "flywheel"));
        vision.init(hardwareMap, telemetry); // Vision uses "Webcam1"
        autoAim.init(alliance);
        autoAim.disable();
        telemetry.addLine("Place turret pointing robot-forward before INIT.");
        telemetry.addLine("A: enable | B: disable | X: RED | Y: BLUE");
        telemetry.addLine("No feeder control. Verify camera geometry first.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Alliance selection resets AutoAim to a disabled state.
        if (gamepad1.x && !previousX) selectAlliance(Vision.Cell.Color.RED);
        if (gamepad1.y && !previousY) selectAlliance(Vision.Cell.Color.BLUE);
        if (gamepad1.b && !previousB) autoAim.disable();
        if (gamepad1.a && !previousA) autoAim.enable();

        // AutoAim itself calls turret/hood/flywheel periodic() when a valid
        // solution exists. Do not call those methods again in this OpMode.
        autoAim.periodic();

        Vision.Cell cell = vision.getBestCell(alliance);
        telemetry.addData("Alliance", alliance);
        telemetry.addData("AutoAim enabled", autoAim.isEnabled());
        telemetry.addData("Calibration verified", AutoAim.CALIBRATION_VERIFIED);
        telemetry.addData("Calibration rows", AutoAim.SHOT_TABLE.length);
        telemetry.addData("Status", autoAim.getStatus());

        if (cell == null) {
            telemetry.addLine("CELL: none currently detected/scorable");
        } else {
            double x = cell.getX();
            double y = cell.getY();
            double z = cell.getZ();
            telemetry.addData("CELL", cell.getName());
            telemetry.addData("Scorable", cell.isScorable());
            telemetry.addData("X / Y / Z (in)", "%.1f / %.1f / %.1f", x, y, z);
            if (Double.isFinite(x) && Double.isFinite(y)) {
                telemetry.addData("Observed horizontal D (in)", "%.1f", Math.hypot(x, y));
                telemetry.addData("Observed bearing (deg)", "%.1f",
                        Math.toDegrees(Math.atan2(x, y)));
            }
        }

        telemetry.addData("Target distance (in)", "%.1f", autoAim.getDistanceInches());
        telemetry.addData("Turret target (deg)", "%.1f", autoAim.getTargetTurretDegrees());
        telemetry.addData("Turret actual (deg)", "%.1f", turret.getAngleDegrees());
        telemetry.addData("Turret at target", turret.isAtTarget());
        telemetry.addData("Hood target (servo)", "%.3f", autoAim.getTargetHoodPosition());
        telemetry.addData("Hood commanded (servo)", "%.3f", hood.getPosition());
        telemetry.addData("Flywheel target (RPM)", "%.0f", autoAim.getTargetFlywheelRPM());
        telemetry.addData("Flywheel actual (RPM)", "%.0f", flywheel.getFlywheelRPM());
        telemetry.addData("Flywheel ready", flywheel.isReadyToShoot());
        telemetry.addData("AutoAim ready", autoAim.isReadyToShoot());
        telemetry.addLine("A enable | B disable | X RED | Y BLUE");
        telemetry.addLine("No POLLEN feeding in this test");
        telemetry.update();

        previousA = gamepad1.a;
        previousB = gamepad1.b;
        previousX = gamepad1.x;
        previousY = gamepad1.y;
    }

    private void selectAlliance(Vision.Cell.Color color) {
        alliance = color;
        autoAim.init(color); // Re-selecting requires A to enable again.
        turret.stop();
        flywheel.stop();
    }

    @Override
    public void stop() {
        autoAim.disable();
        vision.close();
    }
}

