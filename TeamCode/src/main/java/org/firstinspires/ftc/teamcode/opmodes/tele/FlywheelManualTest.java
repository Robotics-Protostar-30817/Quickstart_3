
package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

/**
 * Manual power/encoder test for ONE goBILDA Yellow Jacket
 * Planetary Gear Motor.
 *
 * A: Start flywheel
 * B: Stop flywheel
 * D-pad Up: Increase power by 0.05
 * D-pad Down: Decrease power by 0.05
 *
 * Motor rated speed: 6000 RPM
 * Belt ratio: 1:1
 *
 * This test does not operate the turret, hood, or feeder.
 */
@TeleOp(name = "Flywheel Manual Test", group = "Test")
public final class FlywheelManualTest extends OpMode {

    // Must match the FTC Robot Configuration.
    private static final String MOTOR_NAME = "flywheel";

    private static final double POWER_STEP = 0.05;

    private final Flywheel flywheel = Flywheel.INSTANCE;

    private double selectedPower = 0.30;//initial power setting, it can be increased during testing

    private boolean running = false;

    private boolean previousUp = false;
    private boolean previousDown = false;
    private boolean previousA = false;
    private boolean previousB = false;

    @Override
    public void init() {

        DcMotorEx motor =
                hardwareMap.get(DcMotorEx.class, MOTOR_NAME);

        flywheel.init(motor);
        flywheel.stop();

        running = false;

        telemetry.addLine("Flywheel Manual Test");
        telemetry.addLine("Yellow Jacket 6000 RPM");
        telemetry.addLine("Belt Ratio: 1:1");
        telemetry.addLine("");
        telemetry.addLine("A: START");
        telemetry.addLine("B: STOP");
        telemetry.addLine("D-pad Up/Down: Adjust Power");
        telemetry.update();
    }

    @Override
    public void loop() {

        // Increase selected power.
        if (gamepad1.dpad_up && !previousUp) {
            selectedPower =
                    Math.min(1.0, selectedPower + POWER_STEP);

            if (running) {
                flywheel.setPower(selectedPower);
            }
        }

        // Decrease selected power.
        if (gamepad1.dpad_down && !previousDown) {
            selectedPower =
                    Math.max(0.0, selectedPower - POWER_STEP);

            if (running) {
                flywheel.setPower(selectedPower);
            }
        }

        // Start flywheel.
        if (gamepad1.a && !previousA) {
            running = true;
            flywheel.setPower(selectedPower);
        }

        // Stop flywheel.
        if (gamepad1.b && !previousB) {
            running = false;
            flywheel.stop();
        }

        // Save previous gamepad states.
        previousUp = gamepad1.dpad_up;
        previousDown = gamepad1.dpad_down;
        previousA = gamepad1.a;
        previousB = gamepad1.b;

        flywheel.periodic();

        // Driver Station telemetry.
        telemetry.addLine("=== FLYWHEEL MANUAL TEST ===");

        telemetry.addData(
                "Flywheel",
                running ? "RUNNING" : "STOPPED"
        );

        telemetry.addData(
                "Selected Power",
                "%.2f",
                selectedPower
        );

        telemetry.addData(
                "Commanded Power",
                "%.2f",
                flywheel.getTargetPower()
        );

        telemetry.addData(
                "Encoder Velocity",
                "%.1f ticks/sec",
                flywheel.getVelocity()
        );

        telemetry.addData(
                "Encoder Ticks/Rev",
                "%.2f",
                flywheel.getTicksPerRevolution()
        );

        telemetry.addData(
                "Motor RPM",
                "%.0f",
                flywheel.getMotorRPM()
        );

        telemetry.addData(
                "Flywheel RPM (1:1)",
                "%.0f",
                flywheel.getFlywheelRPM()
        );

        telemetry.addData(
                "Nominal Motor RPM",
                "%.0f",
                Flywheel.RATED_MOTOR_RPM
        );

        telemetry.addLine("");
        telemetry.addLine("A: START   B: STOP");
        telemetry.addLine("D-pad Up/Down: Adjust Power");

        telemetry.update();
    }

    @Override
    public void stop() {
        running = false;
        flywheel.stop();
    }
}
