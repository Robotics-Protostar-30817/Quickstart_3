
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

    private final String logTag;
    public DriveDiagnostics(
            HardwareMap hardwareMap,
            String frontLeftName,
            String frontRightName,
            String backLeftName,
            String backRightName, String tag) {

        frontLeft = hardwareMap.get(
                DcMotorEx.class, frontLeftName);
        frontRight = hardwareMap.get(
                DcMotorEx.class, frontRightName);
        backLeft = hardwareMap.get(
                DcMotorEx.class, backLeftName);
        backRight = hardwareMap.get(
                DcMotorEx.class, backRightName);
        logTag = tag;
    }

    public void log(
            double autoSeconds,
            String state,
            String event,
            Follower follower) {
        PathUtility.info(logTag, PathUtility.robotStatus(autoSeconds,
                state,event,follower));
        PathUtility.info(logTag,PathUtility.motorPower(autoSeconds,state,
                event,frontLeft.getPower(),frontRight.getPower(),
                backLeft.getPower(),backRight.getPower()));


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

        PathUtility.info(logTag,PathUtility.poseDelta(autoSeconds,
                state,before,after));
    }
}

