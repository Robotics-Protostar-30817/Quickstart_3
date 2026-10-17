
package org.firstinspires.ftc.teamcode.pedro.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Reusable drivetrain diagnostic logger.
 *
 * Logs:
 * - Autonomous time and state
 * - Pedro follower busy status
 * - Localized position and heading
 * - Four drivetrain motor power commands
 *
 * Does not modify motor powers, directions, encoders,
 * follower state, or Pinpoint localization.
 */
public final class DriveDiagnostics {

    private static final String TAG = "DriveDiagnostics";

    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;

    public DriveDiagnostics(
            HardwareMap hardwareMap,
            String frontLeftName,
            String frontRightName,
            String backLeftName,
            String backRightName) {

        frontLeft = hardwareMap.get(
                DcMotorEx.class, frontLeftName);
        frontRight = hardwareMap.get(
                DcMotorEx.class, frontRightName);
        backLeft = hardwareMap.get(
                DcMotorEx.class, backLeftName);
        backRight = hardwareMap.get(
                DcMotorEx.class, backRightName);
    }

    public void log(
            double autoSeconds,
            String state,
            String event,
            Follower follower) {

        Pose pose = follower.pose();

        double x = pose == null ? Double.NaN : pose.x();
        double y = pose == null ? Double.NaN : pose.y();
        double heading = pose == null
                ? Double.NaN
                : Math.toDegrees(pose.heading());

        RobotLog.ii(
                TAG,
                "t=%.2f state=%s event=%s busy=%b "
                        + "x=%.3f y=%.3f heading=%.2f "
                        //+PathUtility.formatPose(pose)
                        + "FL=%.3f FR=%.3f BL=%.3f BR=%.3f",
                autoSeconds,
                state,
                event,
                follower.isBusy(),
                //pose,
                x,
                y,
                heading,
                frontLeft.getPower(),
                frontRight.getPower(),
                backLeft.getPower(),
                backRight.getPower()
        );
    }

    /**
     * Logs the immediate pose change across a stop operation.
     */
    public void logPoseDifference(
            double autoSeconds,
            String state,
            Pose before,
            Pose after) {

        if (before == null || after == null) {
            RobotLog.ww(
                    TAG,
                    "t=%.2f state=%s POSE_CHECK unavailable",
                    autoSeconds,
                    state);
            return;
        }

        double headingDelta = Math.atan2(
                Math.sin(after.heading() - before.heading()),
                Math.cos(after.heading() - before.heading())
        );

        RobotLog.ii(
                TAG,
                "t=%.2f state=%s POSE_AFTER_STOP "
                        + "dx=%.4f dy=%.4f dHeading=%.3f",
                autoSeconds,
                state,
                after.x() - before.x(),
                after.y() - before.y(),
                Math.toDegrees(headingDelta)
        );
    }
}

