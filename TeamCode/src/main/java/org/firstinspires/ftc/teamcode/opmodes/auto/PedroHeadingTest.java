package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Displays the robot pose and heading reported by Pedro Pathing.
 *
 * <p>This test uses the same Pedro {@link Follower} and Pinpoint
 * configuration used by the autonomous code. It is intended for
 * measuring robot rotation accurately during vision/turret calibration.</p>
 *
 * <p>Place the robot facing the CELL at the desired starting orientation
 * before starting the OpMode. Record the initial heading, then rotate the
 * robot physically while keeping its center at the same position.</p>
 */
@TeleOp(name = "Pedro Heading Test", group = "Test")
public class PedroHeadingTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        Follower follower = Constants.create(hardwareMap);

        telemetry.addLine("Pedro Heading Test");
        telemetry.addLine("Keep robot stationary until START.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            follower.update();

            double headingRadians = follower.pose().heading();
            double headingDegrees = Math.toDegrees(headingRadians);

            telemetry.addData(
                    "X (in)",
                    "%.2f",
                    follower.pose().x()
            );

            telemetry.addData(
                    "Y (in)",
                    "%.2f",
                    follower.pose().y()
            );

            telemetry.addData(
                    "Heading (rad)",
                    "%.4f",
                    headingRadians
            );

            telemetry.addData(
                    "Heading (deg)",
                    "%.2f",
                    headingDegrees
            );

            telemetry.update();
        }
    }
}