package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;


/**
 * Manual TeleOp test for the {@link Shooter} subsystem.
 *
 * <p>This OpMode is intended for the first-stage testing of the two
 * flywheel motors before a feeder, adjustable hood, or automatic aiming
 * system is added.</p>
 *
 * <p><b>Gamepad controls:</b></p>
 * <ul>
 *     <li><b>A - START:</b> Starts both flywheel motors at the currently
 *     selected power.</li>
 *     <li><b>B - STOP:</b> Stops both flywheel motors.</li>
 *     <li><b>D-pad Up:</b> Increases the selected flywheel power.</li>
 *     <li><b>D-pad Down:</b> Decreases the selected flywheel power.</li>
 * </ul>
 *
 * <p>The Driver Station telemetry displays:</p>
 * <ul>
 *     <li>Whether the shooter is RUNNING or STOPPED.</li>
 *     <li>The selected flywheel power.</li>
 *     <li>The commanded flywheel power.</li>
 *     <li>The measured left flywheel velocity.</li>
 *     <li>The measured right flywheel velocity.</li>
 *     <li>The velocity difference between the two motors.</li>
 * </ul>
 *
 * <p>This test intentionally does not use Pedro Pathing, AprilTag vision,
 * autonomous aiming, or a feeder. Its purpose is to verify the basic
 * flywheel mechanism and motor configuration.</p>
 */
@TeleOp(name = "Shooter Manual Test", group = "Test")
public class ShooterManualTest extends OpMode {

    /**
     * FTC hardware-map name for the left flywheel motor.
     *
     * <p>Change this value if the Control Hub configuration uses a
     * different name.</p>
     */
    private static final String LEFT_MOTOR_NAME = "leftFlywheel";

    /**
     * FTC hardware-map name for the right flywheel motor.
     *
     * <p>Change this value if the Control Hub configuration uses a
     * different name.</p>
     */
    private static final String RIGHT_MOTOR_NAME = "rightFlywheel";

    /**
     * Amount by which the selected flywheel power changes each time
     * D-pad Up or D-pad Down is pressed.
     */
    private static final double POWER_STEP = 0.05;

    /**
     * Initial selected flywheel power.
     *
     * <p>A moderate initial value is used so that the first test does
     * not immediately command full motor power.</p>
     */
    private double selectedPower = 0.30;

    /**
     * Indicates whether the flywheel motors are currently commanded
     * to run.
     */
    private boolean shooterRunning = false;

    /**
     * Stores the previous D-pad Up state so that holding the button
     * does not repeatedly increase the selected power every loop.
     */
    private boolean previousDpadUp = false;

    /**
     * Stores the previous D-pad Down state so that holding the button
     * does not repeatedly decrease the selected power every loop.
     */
    private boolean previousDpadDown = false;


    /**
     * Initializes the shooter subsystem.
     *
     * <p>This method obtains the two flywheel motors from the FTC
     * hardware map and passes them to {@link Shooter#init(DcMotorEx, DcMotorEx)}.
     * The shooter remains stopped after initialization.</p>
     */
    @Override
    public void init() {

        DcMotorEx leftFlywheel =
                hardwareMap.get(DcMotorEx.class, LEFT_MOTOR_NAME);

        DcMotorEx rightFlywheel =
                hardwareMap.get(DcMotorEx.class, RIGHT_MOTOR_NAME);

        Shooter.INSTANCE.init(leftFlywheel, rightFlywheel);
        Shooter.INSTANCE.stop();

        telemetry.addLine("Shooter Manual Test Initialized");
        telemetry.addLine("");
        telemetry.addLine("A = START");
        telemetry.addLine("B = STOP");
        telemetry.addLine("D-pad Up = Increase Power");
        telemetry.addLine("D-pad Down = Decrease Power");
        telemetry.update();
    }


    /**
     * Runs repeatedly while the Shooter Manual Test OpMode is active.
     *
     * <p>The driver can adjust the selected motor power using the D-pad.
     * Pressing A starts the shooter at the selected power, while pressing
     * B stops the shooter.</p>
     *
     * <p>Telemetry continuously displays the selected power, commanded
     * power, measured motor velocities, and the difference between the
     * two measured velocities.</p>
     */
    @Override
    public void loop() {

        /*
         * -------------------------------------------------------------
         * MANUAL SHOOTER CONTROLS
         * -------------------------------------------------------------
         *
         * A = START both flywheel motors.
         * B = STOP both flywheel motors.
         *
         * D-pad Up   = increase selected flywheel power.
         * D-pad Down = decrease selected flywheel power.
         * -------------------------------------------------------------
         */


        // Increase power once for each D-pad Up press.
        if (gamepad1.dpad_up && !previousDpadUp) {

            selectedPower += POWER_STEP;

            if (selectedPower > 1.0) {
                selectedPower = 1.0;
            }

            /*
             * If the shooter is already running, immediately apply
             * the newly selected power.
             */
            if (shooterRunning) {
                Shooter.INSTANCE.setPower(selectedPower);
            }
        }


        // Decrease power once for each D-pad Down press.
        if (gamepad1.dpad_down && !previousDpadDown) {

            selectedPower -= POWER_STEP;

            if (selectedPower < 0.0) {
                selectedPower = 0.0;
            }

            /*
             * If the shooter is already running, immediately apply
             * the newly selected power.
             */
            if (shooterRunning) {
                Shooter.INSTANCE.setPower(selectedPower);
            }
        }


        /*
         * A = START
         *
         * Start both flywheel motors using the currently selected power.
         */
        if (gamepad1.a) {

            shooterRunning = true;
            Shooter.INSTANCE.setPower(selectedPower);
        }


        /*
         * B = STOP
         *
         * Stop both flywheel motors.
         */
        if (gamepad1.b) {

            shooterRunning = false;
            Shooter.INSTANCE.stop();
        }


        /*
         * Save the current D-pad states for edge detection during
         * the next loop iteration.
         */
        previousDpadUp = gamepad1.dpad_up;
        previousDpadDown = gamepad1.dpad_down;


        /*
         * Run the shooter subsystem's periodic processing.
         */
        Shooter.INSTANCE.periodic();


        /*
         * -------------------------------------------------------------
         * DRIVER STATION TELEMETRY
         * -------------------------------------------------------------
         *
         * Display the information needed to evaluate the two flywheel
         * motors during manual testing.
         * -------------------------------------------------------------
         */

        telemetry.addLine("=== SHOOTER MANUAL TEST ===");

        telemetry.addData(
                "Shooter",
                shooterRunning ? "RUNNING" : "STOPPED"
        );

        telemetry.addData(
                "Selected Power",
                "%.2f",
                selectedPower
        );

        telemetry.addData(
                "Commanded Power",
                "%.2f",
                Shooter.INSTANCE.getTargetPower()
        );

        telemetry.addData(
                "Left Velocity",
                "%.1f ticks/sec",
                Shooter.INSTANCE.getLeftVelocity()
        );

        telemetry.addData(
                "Right Velocity",
                "%.1f ticks/sec",
                Shooter.INSTANCE.getRightVelocity()
        );

        telemetry.addData(
                "Velocity Difference",
                "%.1f ticks/sec",
                Shooter.INSTANCE.getVelocityDifference()
        );

        telemetry.addLine("");
        telemetry.addLine("A: START   B: STOP");
        telemetry.addLine("D-pad Up/Down: Adjust Power");

        telemetry.update();
    }


    /**
     * Stops the shooter when the OpMode ends.
     *
     * <p>This ensures that both flywheel motors are commanded to zero
     * when the driver presses STOP on the Driver Station.</p>
     */
    @Override
    public void stop() {
        Shooter.INSTANCE.stop();
    }
}
