
package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * Manual and encoder-relative position test for Turret.java.
 */
@TeleOp(name = "Turret Manual Test", group = "Test")
public class TurretManualTest extends OpMode {

    private static final String MOTOR_NAME = "turret";
    private static final double STICK_DEADBAND = 0.10;

    private final Turret turret = Turret.INSTANCE;

    private double manualPowerLimit = 0.10;
    private boolean wasManual = false;

    private boolean prevA, prevB, prevX, prevY;

    @Override
    public void init() {

        // Placeholder until actual pulley ratio is measured.
        Turret.MOTOR_REVS_PER_TURRET_REV = 3.0;

        turret.init(
                hardwareMap.get(DcMotorEx.class, MOTOR_NAME)
        );

        turret.stop();

        telemetry.addLine(
                "Start with turret centered: startup position = 0 degrees"
        );
        telemetry.addLine("A: 0 | X: -15 | Y: +15 | B: stop");
        telemetry.addLine("Left stick: manual | LB: 10% | RB: 20%");
        telemetry.update();
    }

    @Override
    public void loop() {

        if (gamepad1.left_bumper) {
            manualPowerLimit = 0.10;
        }

        if (gamepad1.right_bumper) {
            manualPowerLimit = 0.20;
        }

        // B is a hold-to-stop override.
        if (gamepad1.b) {

            turret.stop();
            wasManual = false;

        } else {

            double stick = gamepad1.left_stick_x;

            if (Math.abs(stick) > STICK_DEADBAND) {

                turret.setManualPower(
                        stick * manualPowerLimit
                );

                wasManual = true;

            } else {

                // Stop manual movement on joystick release.
                if (wasManual) {
                    turret.stop();
                    wasManual = false;
                }

                if (gamepad1.a && !prevA) {
                    turret.setTargetAngle(0.0);
                }

                if (gamepad1.x && !prevX) {
                    turret.setTargetAngle(-15.0);
                }

                if (gamepad1.y && !prevY) {
                    turret.setTargetAngle(15.0);
                }
            }
        }

        turret.periodic();

        telemetry.addLine("=== TURRET TEST ===");

        telemetry.addData(
                "Current angle",
                "%.2f deg",
                turret.getAngleDegrees()
        );

        telemetry.addData(
                "Target angle",
                "%.2f deg",
                turret.getTargetAngle()
        );

        telemetry.addData(
                "Encoder ticks",
                turret.getEncoderTicks()
        );

        telemetry.addData(
                "Ticks / motor rev",
                "%.2f",
                turret.getTicksPerMotorRevolution()
        );

        telemetry.addData(
                "Motor revs / turret rev",
                "%.2f",
                Turret.MOTOR_REVS_PER_TURRET_REV
        );

        telemetry.addData(
                "Position control",
                turret.isPositionMode()
        );

        telemetry.addData(
                "At target",
                turret.isAtTarget()
        );

        telemetry.addData(
                "Manual power limit",
                "%.0f%%",
                manualPowerLimit * 100.0
        );

        telemetry.addLine(
                "Left stick: rotate | A: 0 | X: -15 | Y: +15"
        );
        telemetry.addLine(
                "B: stop | LB: 10% | RB: 20%"
        );

        telemetry.update();

        prevA = gamepad1.a;
        prevB = gamepad1.b;
        prevX = gamepad1.x;
        prevY = gamepad1.y;
    }

    @Override
    public void stop() {
        turret.stop();
    }
}
