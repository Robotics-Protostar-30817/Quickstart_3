
package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Hood;

/**
 * Manual test for the shooter hood servo.
 *
 * D-pad Up:   Increase servo position by 0.01
 * D-pad Down: Decrease servo position by 0.01
 * A:          Set position to 0.50
 * X:          Set position to 0.45
 * Y:          Set position to 0.55
 * B:          Block new commands while held
 *
 * Servo hardware configuration name: "hood"
 */
@TeleOp(name = "Hood Manual Test", group = "Test")
public class HoodManualTest extends OpMode {

    private static final String SERVO_NAME = "hood";

    private final Hood hood = Hood.INSTANCE;

    private boolean prevUp = false;
    private boolean prevDown = false;
    private boolean prevA = false;
    private boolean prevX = false;
    private boolean prevY = false;

    @Override
    public void init() {

        Servo servo = hardwareMap.get(
                Servo.class,
                SERVO_NAME
        );

        hood.init(servo);

        telemetry.addLine("Hood Manual Test Ready");
        telemetry.addLine("D-pad Up/Down: adjust by 0.01");
        telemetry.addLine("A: 0.50 | X: 0.45 | Y: 0.55");
        telemetry.addLine("Hold B to block new commands");
        telemetry.update();
    }

    @Override
    public void loop() {

        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        boolean a = gamepad1.a;
        boolean x = gamepad1.x;
        boolean y = gamepad1.y;

        // B prevents additional position commands.
        if (!gamepad1.b) {

            if (up && !prevUp) {
                hood.adjustPosition(Hood.TEST_STEP);
            }

            if (down && !prevDown) {
                hood.adjustPosition(-Hood.TEST_STEP);
            }

            if (a && !prevA) {
                hood.setPosition(0.50);
            }

            if (x && !prevX) {
                hood.setPosition(0.45);
            }

            if (y && !prevY) {
                hood.setPosition(0.55);
            }
        }

        hood.periodic();

        telemetry.addLine("=== HOOD MANUAL TEST ===");

        telemetry.addData(
                "Servo position",
                "%.3f",
                hood.getPosition()
        );

        telemetry.addData(
                "Minimum position",
                "%.3f",
                Hood.MIN_POSITION
        );

        telemetry.addData(
                "Maximum position",
                "%.3f",
                Hood.MAX_POSITION
        );

        telemetry.addData(
                "Step",
                "%.3f",
                Hood.TEST_STEP
        );

        telemetry.addData(
                "Commands",
                gamepad1.b ? "BLOCKED" : "ENABLED"
        );

        telemetry.addLine("D-pad Up/Down: adjust servo");
        telemetry.addLine("A: 0.50 | X: 0.45 | Y: 0.55");
        telemetry.addLine("Hold B: block new commands");

        telemetry.update();

        prevUp = up;
        prevDown = down;
        prevA = a;
        prevX = x;
        prevY = y;
    }

    @Override
    public void stop() {
        hood.hold();
    }
}
