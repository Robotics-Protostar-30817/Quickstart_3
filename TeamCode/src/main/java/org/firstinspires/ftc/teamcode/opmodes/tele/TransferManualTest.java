
package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.Transfer;

@TeleOp(name = "Transfer Manual Test", group = "Test")
public class TransferManualTest extends LinearOpMode {

    private double selectedPower = 0.30;
    private static final double POWER_STEP = 0.05;

    private boolean wasXPressed = false;
    private boolean isRunning = false;

    @Override
    public void runOpMode() {

        DcMotorEx transferMotor1 =
                hardwareMap.get(DcMotorEx.class, "transfer1");

        DcMotorEx transferMotor2 =
                hardwareMap.get(DcMotorEx.class, "transfer2");

        Transfer.INSTANCE.init(transferMotor1, transferMotor2);

        telemetry.addLine("Transfer Manual Test Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Start transfer
            if (gamepad1.a) {
                Transfer.INSTANCE.setPower(selectedPower);
                isRunning = true;
            }

            // Stop transfer
            if (gamepad1.b) {
                Transfer.INSTANCE.stop();
                isRunning = false;
            }

            // Increase power
            if (gamepad1.dpad_up) {
                selectedPower = Math.min(1.0, selectedPower + POWER_STEP);

                if (isRunning) {
                    Transfer.INSTANCE.setPower(selectedPower);
                }
            }

            // Decrease power
            if (gamepad1.dpad_down) {
                selectedPower = Math.max(0.0, selectedPower - POWER_STEP);

                if (isRunning) {
                    Transfer.INSTANCE.setPower(selectedPower);
                }
            }

            // Toggle direction
            if (gamepad1.x && !wasXPressed) {
                Transfer.INSTANCE.toggleDirection();

                if (isRunning) {
                    Transfer.INSTANCE.setPower(selectedPower);
                }
            }

            wasXPressed = gamepad1.x;

            Transfer.INSTANCE.periodic();

            telemetry.addData("Status", isRunning ? "RUNNING" : "STOPPED");
            telemetry.addData("Selected Power", selectedPower);
            telemetry.addData("Target Power", Transfer.INSTANCE.getTargetPower());
            telemetry.addData("Motor 1 Velocity", Transfer.INSTANCE.getVelocity1());
            telemetry.addData("Motor 2 Velocity", Transfer.INSTANCE.getVelocity2());
            telemetry.addData("Direction", Transfer.INSTANCE.getDirection());

            telemetry.update();
        }

        Transfer.INSTANCE.stop();
    }

}
