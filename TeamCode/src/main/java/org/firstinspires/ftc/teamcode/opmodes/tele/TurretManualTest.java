package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Turret;


/**
 * Manual calibration and movement test for the Axon MINI Servo MK2
 * shooter turret.
 *
 * <p>This OpMode is intended to characterize the turret mechanism before
 * AprilTag automatic aiming is enabled. It allows the operator to adjust
 * the commanded turret angle and observe the resulting servo position.</p>
 *
 * <p><b>Controls:</b></p>
 * <ul>
 *     <li>D-pad Left: decrease commanded turret angle.</li>
 *     <li>D-pad Right: increase commanded turret angle.</li>
 *     <li>D-pad Up: increase angle step size.</li>
 *     <li>D-pad Down: decrease angle step size.</li>
 *     <li>A: command turret to 0 degrees / center.</li>
 * </ul>
 *
 * <p>The physical turret angle should be measured independently during
 * calibration. The reported angle is the commanded angle, not measured
 * physical feedback from the servo.</p>
 *
 * <p>Start with small movements and verify the physical direction before
 * approaching either mechanical limit.</p>
 */
@TeleOp(name = "Turret Manual Test", group = "Test")
public class TurretManualTest extends OpMode {

    /**
     * FTC Robot Configuration name for the Axon turret servo.
     */
    private static final String TURRET_SERVO_NAME = "turret";

    /**
     * Initial commanded turret angle in degrees.
     */
    private double targetAngleDegrees = 0.0;

    /**
     * Number of degrees added or removed for each D-pad Left/Right press.
     */
    private double angleStepDegrees = 5.0;

    /**
     * Previous D-pad states used for button edge detection.
     */
    private boolean previousLeft = false;
    private boolean previousRight = false;
    private boolean previousUp = false;
    private boolean previousDown = false;
    private boolean previousA = false;


    /**
     * Initializes the Axon MINI Servo MK2 turret.
     *
     * <p>The turret is commanded to its configured center position,
     * corresponding to a target angle of 0 degrees.</p>
     */
    @Override
    public void init() {

        Servo turretServo =
                hardwareMap.get(
                        Servo.class,
                        TURRET_SERVO_NAME
                );

        Turret.INSTANCE.init(turretServo);

        /*
         * Initial calibration values.
         *
         * Change these after measuring the physical turret.
         */
        Turret.INSTANCE.setCenterPosition(0.50);
        Turret.INSTANCE.setDegreesPerServoRange(180.0);
        Turret.INSTANCE.setAngleLimits(-90.0, 90.0);

        targetAngleDegrees = 0.0;

        Turret.INSTANCE.center();

        telemetry.addLine("=== AXON TURRET MANUAL TEST ===");
        telemetry.addLine("");
        telemetry.addLine("LEFT / RIGHT = Change turret angle");
        telemetry.addLine("UP / DOWN = Change angle step");
        telemetry.addLine("A = Center turret at 0 degrees");
        telemetry.addLine("");
        telemetry.addLine("Start with SMALL movements.");
        telemetry.addLine("Measure actual physical angle.");
        telemetry.update();
    }


    /**
     * Handles manual turret commands and displays calibration telemetry.
     */
    @Override
    public void loop() {

        /*
         * =============================================================
         * CHANGE ANGLE STEP
         * =============================================================
         */

        if (gamepad1.dpad_up && !previousUp) {

            angleStepDegrees += 1.0;

            if (angleStepDegrees > 15.0) {
                angleStepDegrees = 15.0;
            }
        }

        if (gamepad1.dpad_down && !previousDown) {

            angleStepDegrees -= 1.0;

            if (angleStepDegrees < 1.0) {
                angleStepDegrees = 1.0;
            }
        }


        /*
         * =============================================================
         * ROTATE LEFT
         * =============================================================
         */

        if (gamepad1.dpad_left && !previousLeft) {

            targetAngleDegrees -= angleStepDegrees;

            Turret.INSTANCE.setTargetAngle(
                    targetAngleDegrees
            );

            /*
             * Read the actual clamped target back from the subsystem.
             */
            targetAngleDegrees =
                    Turret.INSTANCE.getTargetAngle();
        }


        /*
         * =============================================================
         * ROTATE RIGHT
         * =============================================================
         */

        if (gamepad1.dpad_right && !previousRight) {

            targetAngleDegrees += angleStepDegrees;

            Turret.INSTANCE.setTargetAngle(
                    targetAngleDegrees
            );

            targetAngleDegrees =
                    Turret.INSTANCE.getTargetAngle();
        }


        /*
         * =============================================================
         * A = CENTER TURRET
         * =============================================================
         */

        if (gamepad1.a && !previousA) {

            targetAngleDegrees = 0.0;

            Turret.INSTANCE.center();
        }


        /*
         * Save button states for edge detection.
         */
        previousLeft = gamepad1.dpad_left;
        previousRight = gamepad1.dpad_right;
        previousUp = gamepad1.dpad_up;
        previousDown = gamepad1.dpad_down;
        previousA = gamepad1.a;


        /*
         * No periodic control is currently required by Turret because
         * the servo performs its own internal position control.
         */
        Turret.INSTANCE.periodic();


        /*
         * =============================================================
         * TELEMETRY
         * =============================================================
         */

        telemetry.addLine("=== AXON TURRET MANUAL TEST ===");

        telemetry.addData(
                "Commanded Angle",
                "%.1f deg",
                Turret.INSTANCE.getTargetAngle()
        );

        telemetry.addData(
                "Servo Position",
                "%.4f",
                Turret.INSTANCE.getServoPosition()
        );

        telemetry.addData(
                "Angle Step",
                "%.1f deg",
                angleStepDegrees
        );

        telemetry.addLine("");
        telemetry.addLine("LEFT  = decrease angle");
        telemetry.addLine("RIGHT = increase angle");
        telemetry.addLine("UP/DOWN = change angle step");
        telemetry.addLine("A = center");

        telemetry.addLine("");
        telemetry.addLine(
                "Servo position is commanded position;"
        );
        telemetry.addLine(
                "measure physical turret angle separately."
        );

        telemetry.update();
    }


    /**
     * Returns the turret to its center position when the test ends.
     */
    @Override
    public void stop() {

        Turret.INSTANCE.center();
    }
}