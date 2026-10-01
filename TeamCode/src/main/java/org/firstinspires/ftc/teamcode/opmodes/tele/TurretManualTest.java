package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


/**
 * Manual TeleOp test for the turret rotation mechanism.
 *
 * <p>This OpMode is intended for the first mechanical test of the turret.
 * It directly controls the turret motor so that the direction of rotation,
 * encoder direction, mechanical range, and approximate encoder counts per
 * degree can be measured before automatic position control is enabled.</p>
 *
 * <p><b>Gamepad controls:</b></p>
 *
 * <ul>
 *     <li><b>D-pad Left:</b> Rotate the turret in one direction.</li>
 *     <li><b>D-pad Right:</b> Rotate the turret in the opposite direction.</li>
 *     <li><b>Release D-pad:</b> Stop the turret.</li>
 *     <li><b>D-pad Up:</b> Increase manual turret power.</li>
 *     <li><b>D-pad Down:</b> Decrease manual turret power.</li>
 *     <li><b>A:</b> Reset the turret encoder to zero at the current position.</li>
 *     <li><b>B:</b> Emergency/manual stop of turret movement.</li>
 * </ul>
 *
 * <p>The Driver Station telemetry displays the selected motor power,
 * encoder position, encoder velocity, and the current zero reference.</p>
 *
 * <p><b>Important:</b> This test does not enforce software rotation limits.
 * The operator must stop the turret before it reaches a mechanical stop
 * or begins twisting wires. Use low power during the first tests.</p>
 */
@TeleOp(name = "Turret Manual Test", group = "Test")
public class TurretManualTest extends OpMode {

    /**
     * FTC hardware-map name of the turret motor.
     *
     * <p>Change this constant if a different name is used in the
     * Control Hub configuration.</p>
     */
    private static final String TURRET_MOTOR_NAME = "turret";

    /**
     * Amount by which manual turret power changes for each D-pad
     * Up or Down button press.
     */
    private static final double POWER_STEP = 0.05;

    /**
     * Initial turret motor power used for manual rotation.
     *
     * <p>The initial value is intentionally low so that the first
     * mechanical test can be performed slowly.</p>
     */
    private double selectedPower = 0.15;

    /**
     * Turret rotation motor.
     */
    private DcMotorEx turretMotor;

    /**
     * Previous D-pad Up state used for button edge detection.
     */
    private boolean previousDpadUp = false;

    /**
     * Previous D-pad Down state used for button edge detection.
     */
    private boolean previousDpadDown = false;

    /**
     * Previous A-button state used to prevent repeated encoder resets
     * while the button is held.
     */
    private boolean previousA = false;

    /**
     * Indicates whether the B button has requested a manual stop.
     *
     * <p>Moving the D-pad Left or Right clears this flag and allows
     * turret movement again.</p>
     */
    private boolean stoppedByB = false;


    /**
     * Initializes the turret motor for manual testing.
     *
     * <p>The turret should preferably be placed in a known forward
     * position before INIT is pressed. The encoder is reset so that
     * this physical position becomes encoder position zero.</p>
     */
    @Override
    public void init() {

        turretMotor = hardwareMap.get(
                DcMotorEx.class,
                TURRET_MOTOR_NAME
        );

        /*
         * This direction is only an initial setting.
         *
         * During the first test, verify which physical direction
         * corresponds to positive encoder counts.
         */
        turretMotor.setDirection(
                DcMotorSimple.Direction.FORWARD
        );

        /*
         * Reset the encoder so the starting turret position is zero.
         */
        turretMotor.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        /*
         * RUN_USING_ENCODER permits direct motor-power control while
         * allowing encoder position and velocity to be measured.
         */
        turretMotor.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );

        /*
         * BRAKE helps the turret remain near its position when power
         * is removed.
         */
        turretMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        turretMotor.setPower(0.0);

        telemetry.addLine("=== TURRET MANUAL TEST ===");
        telemetry.addLine("");
        telemetry.addLine("D-pad LEFT  = Rotate Left");
        telemetry.addLine("D-pad RIGHT = Rotate Right");
        telemetry.addLine("D-pad UP    = Increase Power");
        telemetry.addLine("D-pad DOWN  = Decrease Power");
        telemetry.addLine("A = Reset Encoder to Zero");
        telemetry.addLine("B = STOP");
        telemetry.addLine("");
        telemetry.addLine("Start with LOW POWER.");
        telemetry.update();
    }


    /**
     * Runs the manual turret control and telemetry.
     *
     * <p>The turret is moved directly using motor power rather than
     * target angles. This allows the mechanical direction, encoder
     * direction, usable rotation range, and gearing to be characterized
     * before the {@code Turret} subsystem's position controller is used.</p>
     */
    @Override
    public void loop() {

        /*
         * =============================================================
         * POWER ADJUSTMENT
         * =============================================================
         */

        if (gamepad1.dpad_up && !previousDpadUp) {

            selectedPower += POWER_STEP;

            if (selectedPower > 0.50) {
                selectedPower = 0.50;
            }
        }

        if (gamepad1.dpad_down && !previousDpadDown) {

            selectedPower -= POWER_STEP;

            if (selectedPower < 0.05) {
                selectedPower = 0.05;
            }
        }


        /*
         * =============================================================
         * A = RESET ENCODER ZERO
         * =============================================================
         *
         * Place the turret at the desired forward/reference position
         * and press A.
         *
         * That physical position becomes encoder position 0.
         */

        if (gamepad1.a && !previousA) {

            turretMotor.setPower(0.0);

            turretMotor.setMode(
                    DcMotor.RunMode.STOP_AND_RESET_ENCODER
            );

            turretMotor.setMode(
                    DcMotor.RunMode.RUN_USING_ENCODER
            );
        }


        /*
         * =============================================================
         * B = STOP
         * =============================================================
         */

        if (gamepad1.b) {
            stoppedByB = true;
        }


        /*
         * =============================================================
         * MANUAL TURRET ROTATION
         * =============================================================
         *
         * D-pad LEFT  = rotate turret left.
         * D-pad RIGHT = rotate turret right.
         *
         * Releasing both buttons stops the turret.
         *
         * If the physical directions are reversed, change the motor
         * direction or reverse the power signs after testing.
         */

        if (gamepad1.dpad_left) {

            stoppedByB = false;

            turretMotor.setPower(-selectedPower);

        } else if (gamepad1.dpad_right) {

            stoppedByB = false;

            turretMotor.setPower(selectedPower);

        } else {

            turretMotor.setPower(0.0);
        }


        /*
         * B always overrides normal turret movement.
         */
        if (stoppedByB) {
            turretMotor.setPower(0.0);
        }


        /*
         * Save button states for edge detection during the next loop.
         */
        previousDpadUp = gamepad1.dpad_up;
        previousDpadDown = gamepad1.dpad_down;
        previousA = gamepad1.a;


        /*
         * =============================================================
         * DRIVER STATION TELEMETRY
         * =============================================================
         *
         * Display the information needed to characterize the turret
         * mechanism before automatic position control is implemented.
         */

        telemetry.addLine("=== TURRET MANUAL TEST ===");

        telemetry.addData(
                "Selected Power",
                "%.2f",
                selectedPower
        );

        telemetry.addData(
                "Motor Power",
                "%.2f",
                turretMotor.getPower()
        );

        telemetry.addData(
                "Encoder Position",
                "%d ticks",
                turretMotor.getCurrentPosition()
        );

        telemetry.addData(
                "Encoder Velocity",
                "%.1f ticks/sec",
                turretMotor.getVelocity()
        );

        telemetry.addData(
                "Stopped by B",
                stoppedByB ? "YES" : "NO"
        );

        telemetry.addLine("");
        telemetry.addLine("LEFT/RIGHT: Rotate Turret");
        telemetry.addLine("UP/DOWN: Adjust Power");
        telemetry.addLine("A: Reset Encoder Zero");
        telemetry.addLine("B: STOP");

        telemetry.update();
    }


    /**
     * Stops the turret motor when the OpMode ends.
     */
    @Override
    public void stop() {

        if (turretMotor != null) {
            turretMotor.setPower(0.0);
        }
    }
}
