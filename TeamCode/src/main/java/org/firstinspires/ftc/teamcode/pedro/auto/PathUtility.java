package org.firstinspires.ftc.teamcode.pedro.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Locale;


public class PathUtility {
    public interface AutoState{
        String name();
    }
    public static String formatPose(Pose pose){
        if (pose== null)
            return "null";
        return String.format(Locale.US,
                "(%.2f, %.2f, %.2f deg",
                pose.x(),pose.y(),Math.toDegrees(pose.heading()));
    }

    /**
     * Writes pose, follower status, and currentState to the robot log.
     * @param event event description
     */
    public static void logPose(String event, Follower follower, ElapsedTime autoTimer,
                         AutoState currentState) {
        if (follower == null) return;
        Pose pose = follower.pose();
        RobotLog.ii("RedScoringAuto",
                "t=%.2f event=%s currentState=%s, pose=%s busy=%b",
                autoTimer.seconds(), event, currentState,
                PathUtility.formatPose(pose), follower.isBusy());
    }
}
