package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import dev.nextftc.core.subsystems.Subsystem;


/**
 * Controls the horizontal rotation of the shooter turret.
 *
 * <p>The turret uses one encoder-equipped motor. Its purpose is to
 * rotate the shooter horizontally so that the shooter points toward
 * the selected CELL opening.</p>
 *
 * <p>The turret angle is measured relative to its initialized
 * forward position. Positive and negative angle directions must be
 * verified on the physical robot.</p>
 *
 * <p>This initial implementation uses proportional position control.
 * More advanced control can be added after the turret mechanics,
 * gear ratio, encoder conversion, and allowable rotation range are
 * measured.</p>
 */
public class Turret implements Subsystem {

    /**
     * Singleton instance of the turret subsystem.
     */
    public static final Turret INSTANCE = new Turret();

    /**
     * Motor that rotates the turret.
     */
    private DcMotorEx turretMotor;

    /**
     * Desired turret angle in degrees.
     */
    private double targetAngleDegrees = 0.0;

    /**
     * Encoder counts corresponding to one degree of turret rotation.
     *
     * <p>This value depends on the motor encoder and mechanical
     * reduction between the motor and turret. It must be measured
     * or calculated after the turret hardware is finalized.</p>
     */
    private double ticksPerDegree = 1.0;

    /**
     * Proportional gain used by the initial turret controller.
     *
     * <p>This is a starting value only and must be tuned on the
     * physical robot.</p>
     */
    private double kP = 0.01;

    /**
     * Maximum absolute motor power permitted during turret movement.
     */
    private double maxPower = 0.40;

    /**
     * Allowed angular error for considering the turret aligned.
     */
    private double angleToleranceDegrees = 1.0;


    /**
     * Creates the singleton turret subsystem.
     */
    private Turret() {
    }


    /**
     * Initializes the turret motor.
     *
     * <p>The turret should be placed in its known forward position
     * before this method is called because the encoder is reset to
     * zero during initialization.</p>
     *
     * @param motor turret rotation motor from the FTC hardware map
     * @param ticksPerDegree encoder counts per degree of actual turret rotation
     */
    public void init(
            DcMotorEx motor,
            double ticksPerDegree) {

        turretMotor = motor;
        this.ticksPerDegree = ticksPerDegree;

        /*
         * Verify direction on the physical robot.
         */
        turretMotor.setDirection(
                DcMotorSimple.Direction.FORWARD
        );

        turretMotor.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        turretMotor.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );

        turretMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        targetAngleDegrees = 0.0;
    }


    /**
     * Sets the desired turret angle relative to the turret's
     * initialized forward position.
     *
     * @param angleDegrees desired turret angle in degrees
     */
    public void setTargetAngle(double angleDegrees) {
        targetAngleDegrees = angleDegrees;
    }


    /**
     * Returns the desired turret angle.
     *
     * @return target turret angle in degrees
     */
    public double getTargetAngle() {
        return targetAngleDegrees;
    }


    /**
     * Returns the turret's current measured angle.
     *
     * @return measured turret angle in degrees
     */
    public double getCurrentAngle() {

        if (turretMotor == null) {
            return 0.0;
        }

        return turretMotor.getCurrentPosition()
                / ticksPerDegree;
    }


    /**
     * Returns the difference between target and measured turret angle.
     *
     * @return turret angular error in degrees
     */
    public double getAngleError() {
        return targetAngleDegrees - getCurrentAngle();
    }


    /**
     * Determines whether the turret is sufficiently close to its
     * requested angle.
     *
     * @return {@code true} when the absolute turret error is within
     *         the configured angular tolerance
     */
    public boolean isAtTarget() {
        return Math.abs(getAngleError())
                <= angleToleranceDegrees;
    }


    /**
     * Immediately stops turret motor output.
     */
    public void stop() {

        if (turretMotor != null) {
            turretMotor.setPower(0.0);
        }
    }


    /**
     * Updates the turret position controller.
     *
     * <p>The controller calculates motor power from the angular
     * difference between the requested and measured turret position.
     * Output is limited to {@link #maxPower}.</p>
     */
    @Override
    public void periodic() {

        if (turretMotor == null) {
            return;
        }

        double error = getAngleError();

        if (Math.abs(error) <= angleToleranceDegrees) {
            turretMotor.setPower(0.0);
            return;
        }

        double power = kP * error;

        power = Math.max(
                -maxPower,
                Math.min(maxPower, power)
        );

        turretMotor.setPower(power);
    }
}
