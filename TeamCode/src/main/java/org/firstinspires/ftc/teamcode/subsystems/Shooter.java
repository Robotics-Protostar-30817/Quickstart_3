package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import dev.nextftc.core.subsystems.Subsystem;

/**
 * Controls the robot's flywheel shooter.
 *
 * <p>The shooter currently consists of two goBILDA 5000 Series
 * 12VDC motors that drive the flywheels. The two motors are controlled
 * together so that both flywheels operate at the same commanded power.</p>
 *
 * <p>This initial version of the subsystem is intended for manual shooter
 * testing. It supports:</p>
 *
 * <ul>
 *     <li>Setting flywheel motor power.</li>
 *     <li>Stopping both flywheel motors.</li>
 *     <li>Reading the encoder velocity of each flywheel motor.</li>
 * </ul>
 *
 * <p>A feeder and an adjustable hood may be added later after the basic
 * flywheel shooter has been mechanically tested. Closed-loop velocity
 * control and automatic shooting will also be added in later development
 * stages.</p>
 */
public class Shooter implements Subsystem {

    /**
     * Singleton instance of the shooter subsystem.
     */
    public static final Shooter INSTANCE = new Shooter();

    /**
     * Left flywheel motor.
     */
    private DcMotorEx leftFlywheel;

    /**
     * Right flywheel motor.
     */
    private DcMotorEx rightFlywheel;

    /**
     * Current commanded motor power.
     *
     * <p>The value is in the range {@code [-1.0, 1.0]}.</p>
     */
    private double targetPower = 0.0;

    /**
     * Creates the singleton shooter subsystem.
     *
     * <p>The constructor is private so that all OpModes use
     * {@link #INSTANCE} rather than creating multiple shooter objects.</p>
     */
    private Shooter() {
    }

    /**
     * Initializes the two flywheel motors from the FTC hardware map.
     *
     * <p>The motor direction settings shown here are initial values only.
     * They must be verified after the physical flywheels are installed.
     * The two flywheels should rotate so that they accelerate the POLLEN
     * toward the shooter exit.</p>
     *
     * @param leftMotor  the left flywheel motor from the hardware map
     * @param rightMotor the right flywheel motor from the hardware map
     */
    public void init(DcMotorEx leftMotor, DcMotorEx rightMotor) {

        leftFlywheel = leftMotor;
        rightFlywheel = rightMotor;

        /*
         * These directions must be verified on the actual robot.
         * Mirrored flywheel motors commonly require opposite motor
         * directions.
         */
        leftFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * RUN_USING_ENCODER allows encoder velocity measurements while
         * still permitting direct motor-power control during early testing.
         */
        leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * Allow the flywheels to coast when motor power is set to zero.
         */
        leftFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        stop();
    }

    /**
     * Sets the power applied to both flywheel motors.
     *
     * <p>This method is intended for the initial manual testing stage.
     * Closed-loop velocity control will be added after the mechanical
     * shooter has been characterized.</p>
     *
     * @param power motor power in the range {@code [-1.0, 1.0]}
     */
    public void setPower(double power) {

        targetPower = Math.max(-1.0, Math.min(1.0, power));

        leftFlywheel.setPower(targetPower);
        rightFlywheel.setPower(targetPower);
    }

    /**
     * Stops both flywheel motors.
     *
     * <p>Because the motors use {@link DcMotor.ZeroPowerBehavior#FLOAT},
     * the flywheels may continue rotating for a short time after this
     * method is called.</p>
     */
    public void stop() {

        targetPower = 0.0;

        if (leftFlywheel != null) {
            leftFlywheel.setPower(0.0);
        }

        if (rightFlywheel != null) {
            rightFlywheel.setPower(0.0);
        }
    }

    /**
     * Returns the currently commanded flywheel motor power.
     *
     * @return commanded motor power in the range {@code [-1.0, 1.0]}
     */
    public double getTargetPower() {
        return targetPower;
    }

    /**
     * Returns the measured velocity of the left flywheel motor.
     *
     * <p>The FTC SDK reports this value in encoder ticks per second.</p>
     *
     * @return left flywheel encoder velocity in ticks per second
     */
    public double getLeftVelocity() {

        if (leftFlywheel == null) {
            return 0.0;
        }

        return leftFlywheel.getVelocity();
    }

    /**
     * Returns the measured velocity of the right flywheel motor.
     *
     * <p>The FTC SDK reports this value in encoder ticks per second.</p>
     *
     * @return right flywheel encoder velocity in ticks per second
     */
    public double getRightVelocity() {

        if (rightFlywheel == null) {
            return 0.0;
        }

        return rightFlywheel.getVelocity();
    }

    /**
     * Returns the absolute difference between the measured velocities
     * of the two flywheel motors.
     *
     * <p>This value is useful during manual testing for identifying
     * significant differences between the two flywheel speeds.</p>
     *
     * @return absolute velocity difference in encoder ticks per second
     */
    public double getVelocityDifference() {
        return Math.abs(
                getLeftVelocity() - getRightVelocity()
        );
    }

    /**
     * Performs periodic shooter subsystem processing.
     *
     * <p>No periodic control is required during the initial power-control
     * testing stage. This method is retained so that closed-loop velocity
     * control and shooter monitoring can be added later without changing
     * the subsystem architecture.</p>
     */
    @Override
    public void periodic() {
        // No periodic control is required during initial manual testing.
    }
}