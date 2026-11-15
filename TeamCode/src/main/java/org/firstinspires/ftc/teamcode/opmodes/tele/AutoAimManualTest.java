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
 * Tests stationary vision-guided aiming WITHOUT feeding or shooting POLLEN.
 *
 * Gamepad 1:
 *   A: enable automatic aiming
 *   B: disable aiming (stops turret/flywheel; hood holds position)
 *   X: select RED alliance (disables aiming)
 *   Y: select BLUE alliance (disables aiming)
 *
 * The hood's angle calibration and the shot table must both contain actual
 * measurements and be verified before AutoAim can command the mechanism.
 * A standard positional servo does not report its physical shaft angle;
 * hood readiness is only a time-based estimate.
 */
@TeleOp(name = "Auto Aim Manual Test", group = "Test")
public final class AutoAimManualTest extends OpMode {
    private final AutoAim aim = AutoAim.INSTANCE;
    private final Turret turret = Turret.INSTANCE;
    private final Hood hood = Hood.INSTANCE;
    private final Flywheel flywheel = Flywheel.INSTANCE;
    private final Vision vision = Vision.INSTANCE;

    private Vision.Cell.Color alliance = Vision.Cell.Color.RED;
    private boolean prevA, prevB, prevX, prevY;

    @Override
    public void init() {
        // Verify these hardware configuration names on the Driver Station.
        turret.init(hardwareMap.get(DcMotorEx.class, "turret"));
        hood.init(hardwareMap.get(Servo.class, "hood"));
        flywheel.init(hardwareMap.get(DcMotorEx.class, "flywheel"));
        vision.init(hardwareMap, telemetry); // Vision uses Webcam1 in current subsystem.
        aim.init(alliance);

        telemetry.addLine("A: enable aim | B: disable aim");
        telemetry.addLine("X: RED | Y: BLUE (selection disables aim)");
        telemetry.addLine("No transfer motors are commanded by this OpMode.");
        telemetry.addLine("Remove POLLEN and verify turret/hood clearance before testing.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Edge-trigger buttons so holding a button does not repeat its action.
        if (gamepad1.b && !prevB) aim.disable();
        if (gamepad1.x && !prevX) selectAlliance(Vision.Cell.Color.RED);
        if (gamepad1.y && !prevY) selectAlliance(Vision.Cell.Color.BLUE);
        if (gamepad1.a && !prevA) aim.enable();

        // AutoAim owns the periodic updates of Turret, Hood and Flywheel.
        aim.periodic();

        Vision.Cell cell = vision.getBestCell(alliance);
        boolean turretReady = aim.isEnabled() && turret.isAtTarget();
        boolean flywheelReady = aim.isEnabled() && flywheel.isReadyToShoot();
        boolean hoodReady = aim.isEnabled() && hood.isReady();

        telemetry.addLine("=== AUTO AIM MANUAL TEST ===");
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Aim enabled", aim.isEnabled());
        telemetry.addData("Aim status", aim.getStatus());
        telemetry.addData("Shot table verified", AutoAim.CALIBRATION_VERIFIED);
        telemetry.addData("Hood angle calibration verified", hood.hasValidAngleCalibration());
        telemetry.addData("CELL", cell == null ? "none" : cell.getName());
        telemetry.addData("CELL scorable", cell != null && cell.isScorable());
        telemetry.addData("Distance (in)", "%.2f", aim.getDistanceInches());

        telemetry.addLine("--- Turret ---");
        telemetry.addData("Target angle (deg)", "%.2f", aim.getTargetTurretDegrees());
        telemetry.addData("Actual angle (deg)", "%.2f", turret.getAngleDegrees());
        telemetry.addData("Turret ready", turretReady);

        telemetry.addLine("--- Hood ---");
        telemetry.addData("Target launch angle (deg)", "%.2f", aim.getTargetHoodAngleDegrees());
        telemetry.addData("Converted servo target", "%.4f", aim.getTargetHoodPosition());
        telemetry.addData("Servo commanded position", "%.4f", hood.getPosition());
        telemetry.addData("Hood ready (TIME ESTIMATE)", hoodReady);
        telemetry.addData("Hood settling seconds", "%.2f", Hood.SETTLE_SECONDS);
        telemetry.addLine("Servo does NOT provide measured physical hood angle.");

        telemetry.addLine("--- Flywheel ---");
        telemetry.addData("Target RPM", "%.1f", aim.getTargetFlywheelRPM());
        telemetry.addData("Actual RPM", "%.1f", flywheel.getFlywheelRPM());
        telemetry.addData("Flywheel ready", flywheelReady);

        telemetry.addLine("--- Combined readiness ---");
        telemetry.addData("Turret / Flywheel / Hood", "%s / %s / %s",
                turretReady, flywheelReady, hoodReady);
        telemetry.addData("AutoAim ready", aim.isReadyToShoot());
        telemetry.addLine("A enable | B disable | X RED | Y BLUE");
        telemetry.update();

        prevA = gamepad1.a;
        prevB = gamepad1.b;
        prevX = gamepad1.x;
        prevY = gamepad1.y;
    }

    private void selectAlliance(Vision.Cell.Color color) {
        aim.disable();
        alliance = color;
        aim.init(color);
    }

    @Override
    public void stop() {
        aim.disable();
        vision.close();
    }
}
