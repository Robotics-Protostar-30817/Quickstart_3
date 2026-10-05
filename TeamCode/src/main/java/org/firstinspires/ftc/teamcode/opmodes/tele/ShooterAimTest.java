package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;


/**
 * Tests coordinated AprilTag-based aiming of the shooter and Axon
 * MINI Servo MK2 turret.
 *
 * <p>The robot must already be positioned inside the intended shooting
 * area. This OpMode does not control the drivetrain.</p>
 *
 * <p>The shooting preparation sequence is:</p>
 * <ol>
 *     <li>Vision detects a scorable AprilTag Cluster CELL.</li>
 *     <li>The CELL bearing determines the turret angle.</li>
 *     <li>The CELL range determines the required flywheel velocity.</li>
 *     <li>The turret is given time to reach its commanded position.</li>
 *     <li>The flywheel must reach its target velocity.</li>
 *     <li>Telemetry reports READY TO SHOOT.</li>
 * </ol>
 *
 * <p>The current test does not feed a POLLEN into the flywheel because
 * a feeder mechanism has not yet been defined.</p>
 */
@TeleOp(name = "Shooter Aim Test", group = "Test")
public class ShooterAimTest extends OpMode {

    /**
     * FTC Robot Configuration name of the flywheel motor.
     */
    private static final String SHOOTER_MOTOR_NAME =
            "shooter";

    /**
     * FTC Robot Configuration name of the Axon turret servo.
     */
    private static final String TURRET_SERVO_NAME =
            "turret";

    /**
     * Approximate time allowed for the turret servo to move to its
     * requested angle before it is considered settled.
     *
     * <p>This is an initial test value and should be replaced with a
     * measured value for the actual turret mechanism.</p>
     */
    private static final double TURRET_SETTLE_TIME_SECONDS =
            0.50;

    /**
     * Minimum range for which shooter calibration data is available,
     * in inches.
     */
    private static final double MIN_SHOOTING_RANGE =
            40.0;

    /**
     * Maximum range for which shooter calibration data is available,
     * in inches.
     */
    private static final double MAX_SHOOTING_RANGE =
            70.0;

    /**
     * Experimentally measured shooting ranges in inches.
     *
     * <p>These initial values are placeholders and must be replaced
     * with measured shooter data.</p>
     */
    private static final double[] RANGE_INCHES = {
            40.0,
            50.0,
            60.0,
            70.0
    };

    /**
     * Flywheel encoder velocities corresponding to
     * {@link #RANGE_INCHES}.
     *
     * <p>These values are placeholders only and must not be treated
     * as calibrated shooting values.</p>
     */
    private static final double[] VELOCITY_TICKS_PER_SECOND = {
            2800.0,
            3100.0,
            3400.0,
            3700.0
    };



    /**
     * Timer measuring how long the turret has been moving toward
     * the currently commanded position.
     */
    private final ElapsedTime turretTimer =
            new ElapsedTime();

    /**
     * Whether the system is currently preparing a shot.
     */
    private boolean aiming = false;

    /**
     * Previous A-button state for edge detection.
     */
    private boolean previousA = false;

    /**
     * Previous B-button state for edge detection.
     */
    private boolean previousB = false;

    /**
     * CELL selected for the current shot.
     */
    private Vision.Cell targetCell = null;


    /**
     * Initializes Vision, the flywheel shooter, and the Axon turret.
     */
    @Override
    public void init() {

        Vision.INSTANCE.init(
                hardwareMap,
                telemetry
        );

        DcMotorEx shooterMotor =
                hardwareMap.get(
                        DcMotorEx.class,
                        SHOOTER_MOTOR_NAME
                );

        Shooter.INSTANCE.init(
                shooterMotor
        );

        Servo turretServo =
                hardwareMap.get(
                        Servo.class,
                        TURRET_SERVO_NAME
                );

        Turret.INSTANCE.init(
                turretServo
        );


        /*
         * =============================================================
         * TURRET CALIBRATION
         * =============================================================
         *
         * These are initial placeholder values.
         *
         * Replace them with measurements obtained from
         * TurretManualTest.
         */

        Turret.INSTANCE.setCenterPosition(0.50);

        Turret.INSTANCE.setDegreesPerServoRange(
                180.0
        );

        Turret.INSTANCE.setAngleLimits(
                -90.0,
                90.0
        );

        Turret.INSTANCE.center();


        telemetry.addLine("=== SHOOTER AIM TEST ===");
        telemetry.addLine("");
        telemetry.addLine("Robot must already be in shooting area.");
        telemetry.addLine("");
        telemetry.addLine("A = Acquire CELL and prepare shot");
        telemetry.addLine("B = Cancel / Stop");
        telemetry.addLine("");
        telemetry.addLine("Feeder is NOT controlled.");
        telemetry.update();
    }


    /**
     * Detects a CELL, aims the turret, commands the required flywheel
     * velocity, and determines when the shooter is ready.
     */
    @Override
    public void loop() {

        Vision.Cell visibleCell =
                Vision.INSTANCE.getBestCell();


        /*
         * =============================================================
         * A = ACQUIRE CELL AND PREPARE SHOT
         * =============================================================
         */

        if (gamepad1.a && !previousA) {

            if (visibleCell != null
                    && visibleCell.isScorable()
                    && isRangeValid(
                    visibleCell.getRange()
            )) {

                targetCell = visibleCell;

                aiming = true;


                /*
                 * -----------------------------------------------------
                 * TURRET AIMING
                 * -----------------------------------------------------
                 *
                 * The AprilTag Cluster bearing is measured relative to
                 * the robot-mounted camera.
                 *
                 * For the initial test, use that bearing directly as
                 * the desired turret angle.
                 *
                 * If the physical turret turns in the opposite direction,
                 * correct the sign after TurretManualTest.
                 */

                //Turret.INSTANCE.setTargetAngle(
                //        targetCell.getBearing()
                //);
                double turretAngle = Turret.INSTANCE.calculateTurretAngle(targetCell);
                Turret.INSTANCE.setTargetAngle(turretAngle);

                /*
                 * Start the settling timer after changing turret angle.
                 */
                turretTimer.reset();


                /*
                 * -----------------------------------------------------
                 * SHOOTER VELOCITY
                 * -----------------------------------------------------
                 *
                 * Select flywheel velocity from the AprilTag-measured
                 * range to the CELL.
                 */

                double shootingDistance = calculateHorizontalDistance(targetCell);
                double targetVelocity =
                        getVelocityForRange(shootingDistance);

                Shooter.INSTANCE.setVelocity(
                        targetVelocity
                );
            }
        }


        /*
         * =============================================================
         * B = CANCEL SHOT
         * =============================================================
         */

        if (gamepad1.b && !previousB) {

            aiming = false;

            targetCell = null;

            Shooter.INSTANCE.stop();

            Turret.INSTANCE.center();
        }


        /*
         * Save button states for edge detection.
         */
        previousA = gamepad1.a;
        previousB = gamepad1.b;


        /*
         * No active software position controller is required for the
         * Axon servo turret, but keep the subsystem lifecycle call.
         */
        Turret.INSTANCE.periodic();


        /*
         * =============================================================
         * READINESS CHECKS
         * =============================================================
         */

        boolean turretReady =
                aiming
                        && turretTimer.seconds()
                        >= TURRET_SETTLE_TIME_SECONDS;

        boolean shooterReady =
                aiming
                        && Shooter.INSTANCE.isAtSpeed();

        boolean targetStillValid =
                aiming
                        && targetCell != null;

        boolean readyToShoot =
                targetStillValid
                        && turretReady
                        && shooterReady;


        /*
         * =============================================================
         * VISION TELEMETRY
         * =============================================================
         */

        telemetry.addLine(
                "=== SHOOTER AIM TEST ==="
        );

        if (visibleCell == null) {

            telemetry.addLine(
                    "CELL: NOT DETECTED"
            );

        } else {

            telemetry.addData(
                    "Visible CELL",
                    visibleCell.getName()
            );

            telemetry.addData("CELL X", "%.1f in",visibleCell.getX());
            telemetry.addData("CELL Y", "%.1f in",visibleCell.getY());
            telemetry.addData("CELL Z", "%.1f in",visibleCell.getZ());

            telemetry.addData(
                    "Scorable",
                    visibleCell.isScorable()
                            ? "YES"
                            : "NO"
            );

            telemetry.addData(
                    "Range",
                    "%.1f in",
                    visibleCell.getRange()
            );

            telemetry.addData(
                    "Bearing",
                    "%.1f deg",
                    visibleCell.getBearing()
            );

            telemetry.addData(
                    "Elevation",
                    "%.1f deg",
                    visibleCell.getElevation()
            );

            telemetry.addData("Calculated Turret Angle",
                    "%.1f deg",Turret.INSTANCE.calculateTurretAngle(visibleCell));
        }


        /*
         * =============================================================
         * TURRET TELEMETRY
         * =============================================================
         */

        telemetry.addLine("");

        telemetry.addData(
                "Aiming",
                aiming ? "YES" : "NO"
        );

        telemetry.addData(
                "Turret Target",
                "%.1f deg",
                Turret.INSTANCE.getTargetAngle()
        );

        telemetry.addData(
                "Turret Servo Position",
                "%.4f",
                Turret.INSTANCE.getServoPosition()
        );

        telemetry.addData(
                "Turret Settle Time",
                "%.2f sec",
                turretTimer.seconds()
        );

        telemetry.addData(
                "Turret Ready",
                turretReady ? "YES" : "NO"
        );


        /*
         * =============================================================
         * SHOOTER TELEMETRY
         * =============================================================
         */

        telemetry.addLine("");

        telemetry.addData(
                "Target Velocity",
                "%.1f ticks/sec",
                Shooter.INSTANCE.getTargetVelocity()
        );

        telemetry.addData(
                "Measured Velocity",
                "%.1f ticks/sec",
                Shooter.INSTANCE.getVelocity()
        );

        telemetry.addData(
                "Shooter Ready",
                shooterReady ? "YES" : "NO"
        );


        /*
         * =============================================================
         * FINAL STATUS
         * =============================================================
         */

        telemetry.addLine("");

        if (readyToShoot) {

            telemetry.addLine(
                    "*** READY TO SHOOT ***"
            );

        } else {

            telemetry.addLine(
                    "NOT READY"
            );
        }

        telemetry.addLine("");

        telemetry.addLine(
                "A = Acquire / Aim"
        );

        telemetry.addLine(
                "B = Cancel / Stop"
        );

        telemetry.update();
    }

    /**
     * Determines whether the detected CELL is within the currently
     * calibrated shooter range.
     *
     * @param rangeInches AprilTag-measured range in inches
     * @return {@code true} if the range lies inside the calibrated
     *         shooting interval
     */
    private boolean isRangeValid(
            double rangeInches) {

        return rangeInches >= MIN_SHOOTING_RANGE
                && rangeInches <= MAX_SHOOTING_RANGE;
    }

    /**
     * Calculate horizontal distance from the turret rotation axis to the detected CELL center.
     *
     * @param targetCell detected CELL with camera-relative position in inches.
     * @return horizontal distance from the turret-to-CELL in inches.
     */
    private double calculateHorizontalDistance(Vision.Cell targetCell){
        double x = Vision.CAMERA_TO_CENTER_X_INCHES + targetCell.getX();
        double y = Vision.CAMERA_TO_CENTER_Y_INCHES + targetCell.getY();
        return Math.hypot(x,y);
    }

    /**
     * Calculates the desired flywheel velocity for a measured CELL range.
     *
     * <p>Linear interpolation is performed between adjacent experimentally
     * measured range/velocity calibration points.</p>
     *
     * @param rangeInches AprilTag-measured range to the CELL in inches
     * @return target flywheel encoder velocity in ticks per second
     */
    private double getVelocityForRange(
            double rangeInches) {

        if (rangeInches <= RANGE_INCHES[0]) {
            return VELOCITY_TICKS_PER_SECOND[0];
        }

        int last =
                RANGE_INCHES.length - 1;

        if (rangeInches >= RANGE_INCHES[last]) {
            return VELOCITY_TICKS_PER_SECOND[last];
        }

        for (int i = 0; i < last; i++) {

            double lowerRange =
                    RANGE_INCHES[i];

            double upperRange =
                    RANGE_INCHES[i + 1];

            if (rangeInches >= lowerRange
                    && rangeInches <= upperRange) {

                double fraction =
                        (rangeInches - lowerRange)
                                / (upperRange - lowerRange);

                double lowerVelocity =
                        VELOCITY_TICKS_PER_SECOND[i];

                double upperVelocity =
                        VELOCITY_TICKS_PER_SECOND[i + 1];

                return lowerVelocity
                        + fraction
                        * (upperVelocity - lowerVelocity);
            }
        }

        return VELOCITY_TICKS_PER_SECOND[last];
    }


    /**
     * Stops the shooter, centers the turret, and closes Vision when
     * the OpMode ends.
     */
    @Override
    public void stop() {

        Shooter.INSTANCE.stop();

        Turret.INSTANCE.center();

        Vision.INSTANCE.close();
    }
}