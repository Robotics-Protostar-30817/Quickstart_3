package org.firstinspires.ftc.teamcode.pedro.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Locale;


public class PathUtility {

    public static final String Default_Tag = "AutoPath";

    public PathUtility(){}

    public interface AutoState{
        String name();
    }

    /**
     * Display position information
     * @param pose
     * @return
     */
    public static String formatPose(Pose pose){
        if (pose== null)
            return "null";
        return String.format(Locale.US,
                "(%.2f, %.2f, %.2f deg)",
                pose.x(),pose.y(),Math.toDegrees(pose.heading()));
    }

    /**
     * Display Robot current status
     */
    public static String robotStatus(double time,
                                     Object state,
                                     String event,
                                     Follower follower){
        return String.format(Locale.US,
                "t=%.2f event=%s state=%s pose=%s, busy=%b",
                time,event,state,formatPose(follower.pose()),
                follower.isBusy());
    }

    /**
     * Returns the shortest signed angular difference in radians.
     * @param a first angle
     * @param b second angle
     * @return signed angular difference in [-pi, pi]
     */
    public static double angleDifference(double a, double b) {
        return Math.atan2(Math.sin(a - b), Math.cos(a - b));
    }
    /**
     * Display Robot state change
     */
    public static String stateChange(double time, Object from,
                                    Object to, String reason)
    {
        return String.format(Locale.US,
                "t=%.2f State from=%s to=%s reason=%s",
                time,from,to,reason);
    }

    /**
     * Display Robot Path information
     */
    public static String pathStart(double time,String name, Pose target){
        return String.format(Locale.US,"t=%.2f Path %s start at %s",
                time,name,formatPose(target));
    }

    /**
     * Display time deadline information
     */
    public static String deadline(double time, double remaining,
                                  double reserve){
        return String.format(Locale.US,
                "t=%.2f Emergency Park remaining=%.2f reserve=%.2f",
                time,remaining,reserve);
    }

    /**
     * Computes the shortest signed heading difference.
     * @param time
     * @param state
     * @param before pose of before
     * @param after pose of after
     * @return difference in [-pi, pi]
     */
    public static String poseDelta(double time, Object state,
                                   Pose before, Pose after) {
        if (before==null ||after==null){
            return String.format(Locale.US,"t=%.2f State=%s POSE_DELTA Unavailable",
                    time,state);
        }
        double d= Math.atan2(
                Math.sin(after.heading() - before.heading()),
                Math.cos(after.heading() - before.heading()));

        return String.format(Locale.US,
                "t=%.2f State=%s POSE_DELTA dx=%.4f, dy=%.4f dHeading=%.2f",
                time,state,after.x()-before.x(),
                after.y()-before.y(),Math.toDegrees(d));
    }

    public static void info(String tag, String message){
        RobotLog.ii(tag,"%s",message);

    }
    public static void warning(String tag, String message){
        RobotLog.ww(tag,"%s",message);
    }
    public static void error(String tag, String message){
        RobotLog.ee(tag,"%s",message);
    }


    /**
     * Writes pose, follower status, and currentState to the robot log.
     * @param event event description
     */
    public static void logPose(String tag,
                               String event,
                               Follower follower,
                               ElapsedTime autoTimer,
                         AutoState currentState) {
        //if (follower == null) return "null";
        info(tag,robotStatus(autoTimer.seconds(),
                currentState,event,follower));
    }

    /**
     * Display motor power information
     */
    public static String motorPower(double time, Object state,String event,double fl, double fr, double bl, double br){
        return String.format(Locale.US,"t=%.2f event=%s state=%s fl=%.2f fr=%.2f bl=%.2f br=%.2f",
                time,event,state,fl,fr,bl,br);
    }
}
