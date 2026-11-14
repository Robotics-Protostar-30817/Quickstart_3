
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import dev.nextftc.core.subsystems.Subsystem;

/**
 * Transfer subsystem
 *
 * Controls the two transfer motors.
 */
public class Transfer implements Subsystem {

    public static final Transfer INSTANCE = new Transfer();

    private DcMotorEx transferMotor1;
    private DcMotorEx transferMotor2;

    private DcMotorSimple.Direction transferDirection =
            DcMotorSimple.Direction.FORWARD;

    private double targetPower = 0.0;

    private Transfer() {
    }

    /**
     * Initializes both transfer motors.
     */
    public void init(DcMotorEx motor1, DcMotorEx motor2) {
        if (motor1 == null || motor2 == null) {
            throw new IllegalArgumentException("Transfer motors cannot be null");
        }

        transferMotor1 = motor1;
        transferMotor2 = motor2;

        transferMotor1.setDirection(transferDirection);
        transferMotor2.setDirection(transferDirection);

        transferMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transferMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        transferMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stop();
    }

    /**
     * Sets the power of both transfer motors.
     */
    public void setPower(double power) {
        if (transferMotor1 == null || transferMotor2 == null) {
            throw new IllegalStateException("Transfer has not been initialized");
        }

        if (!Double.isFinite(power)) {
            throw new IllegalArgumentException("Power must be a finite number");
        }

        targetPower = Math.max(-1.0, Math.min(1.0, power));

        transferMotor1.setPower(targetPower);
        transferMotor2.setPower(targetPower);
    }

    /**
     * Toggles the direction of both transfer motors.
     */
    public void toggleDirection() {
        stop();

        transferDirection =
                (transferDirection == DcMotorSimple.Direction.FORWARD)
                        ? DcMotorSimple.Direction.REVERSE
                        : DcMotorSimple.Direction.FORWARD;

        transferMotor1.setDirection(transferDirection);
        transferMotor2.setDirection(transferDirection);
    }

    /**
     * Stops both transfer motors.
     */
    public void stop() {
        targetPower = 0.0;

        if (transferMotor1 != null) {
            transferMotor1.setPower(0.0);
        }

        if (transferMotor2 != null) {
            transferMotor2.setPower(0.0);
        }
    }

    public double getTargetPower() {
        return targetPower;
    }

    public double getVelocity1() {
        return transferMotor1 == null ? 0.0 : transferMotor1.getVelocity();
    }

    public double getVelocity2() {
        return transferMotor2 == null ? 0.0 : transferMotor2.getVelocity();
    }

    public String getDirection() {
        return transferDirection.toString();
    }

    @Override
    public void periodic() {
    }
}
