package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import dev.nextftc.core.subsystems.Subsystem;


/**
 * Controls the robot's flywheel shooter.
 *
 * <p>The current shooter design uses one goBILDA 5000 Series motor
 * to drive the flywheel. The hood angle is mechanically fixed for
 * the initial design, and the feeder mechanism has not yet been
 * selected.</p>
 *
 * <p>This subsystem initially provides direct power control for
 * manual shooter testing. Closed-loop velocity control will be
 * added after the mechanical shooter has been tested and its
 * operating velocity has been characterized.</p>
 */
public class Shooter implements Subsystem {

    /**
     * Singleton instance of the shooter subsystem.
     */
    public static final Shooter INSTANCE = new Shooter();

    /**
     * Motor that drives the shooter flywheel.
     */
    private DcMotorEx flywheelMotor;

    /**
     * Currently commanded flywheel motor power.
     */
    private double targetPower = 0.0;

    /**
     * Desired flywheel encoder velocity in ticks per second
     */
    private double targetVelocity = 0.0;

    /**
     * Allowed velocity error for considering the flywheel ready to shoot,
     * in encoder ticks per second
     */
    private double velocityTolerance=100.0;

    /**
     * Creates the singleton shooter subsystem.
     */
    private Shooter() {
    }


    /**
     * Initializes the flywheel motor.
     *
     * <p>The motor direction must be verified after the physical
     * shooter is assembled.</p>
     *
     * @param motor flywheel motor from the FTC hardware map
     */
    public void init(DcMotorEx motor) {

        flywheelMotor = motor;

        /*
         * Verify this direction on the physical robot.
         */
        flywheelMotor.setDirection(
                DcMotorSimple.Direction.FORWARD
        );

        /*
         * Use the encoder so that flywheel velocity can be measured.
         */
        flywheelMotor.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );

        /*
         * Allow the flywheel to coast after power is removed.
         */
        flywheelMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        stop();
    }


    /**
     * Sets flywheel motor power.
     *
     * <p>This method is intended primarily for early manual testing.
     * The requested value is limited to the valid FTC motor-power
     * range.</p>
     *
     * @param power requested motor power in the range {@code [-1.0, 1.0]}
     */
    public void setPower(double power) {

        targetPower =
                Math.max(-1.0, Math.min(1.0, power));

        flywheelMotor.setPower(targetPower);
    }

    /**
     * Commands that flywheel to run at a specified encoder velocity.
     *
     * <p>This method uses the FTC motor controller's velocity control.
     * The requested value is expressed in encoder ticks per second.</p>
     *
     * @param velocityTicksPerSecond desired flywheel velocity in encoder ticks per second
     */
     public void setVelocity(double velocityTicksPerSecond){
         targetVelocity = velocityTicksPerSecond;
         flywheelMotor.setVelocity(targetVelocity);
     }

    /**
     * Returns the currently requested flywheel velocity.
     *
     * @return target flywheel velocity in encoder ticks per second
     */
    public double getTargetVelocity(){
        return targetVelocity;
    }

    /**
     * Determines whether the flywheel has reached approximately the requested shooting velocity.
     *
     * @return {@code true} if measured velocity is within the configured
     * tolerance of the target velocity
     */
    public boolean isAtSpeed(){
        if(flywheelMotor == null || targetVelocity <=0.0){
            return false;
        }
        return Math.abs(getVelocity()-targetVelocity)<=velocityTolerance;
    }
    /**
     * Stops the flywheel motor.
     *
     * <p>The flywheel may continue rotating temporarily because
     * zero-power behavior is configured as FLOAT.</p>
     */
    public void stop() {

        targetPower = 0.0;
        targetVelocity = 0.0;

        if (flywheelMotor != null) {
            flywheelMotor.setPower(0.0);
        }
    }


    /**
     * Returns the currently commanded motor power.
     *
     * @return commanded flywheel power in the range {@code [-1.0, 1.0]}
     */
    public double getTargetPower() {
        return targetPower;
    }


    /**
     * Returns the measured flywheel motor velocity.
     *
     * @return encoder velocity in ticks per second
     */
    public double getVelocity() {

        if (flywheelMotor == null) {
            return 0.0;
        }

        return flywheelMotor.getVelocity();
    }


    /**
     * Performs periodic shooter processing.
     *
     * <p>No periodic closed-loop processing is required during the
     * initial direct-power testing stage.</p>
     */
    @Override
    public void periodic() {
        // Closed-loop velocity control will be added later.
    }
}