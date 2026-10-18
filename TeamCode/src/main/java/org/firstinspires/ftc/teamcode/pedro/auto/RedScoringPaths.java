package org.firstinspires.ftc.teamcode.pedro.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Navigation waypoints for the Red Scoring autonomous.
 *
 * <p>IMPORTANT: The strategy image has no coordinate/heading labels. The poses
 * below are estimates from the 144-inch field grid, not measured instructions.
 * Verify them against the field and robot orientation BEFORE enabling motion.
 * The diagonal parking segment has not been collision-clearance certified.
 */
public final class RedScoringPaths {
    private final PoseFactory p = PoseFactory.degrees();

    // Estimated image positions: left green point, green/purple junction,
    // upper green point, and upper-left purple point respectively.
    public final Pose startPose = p.of(65.0, 132.0, 270.0);
    public final Pose scoringPose = p.of(65.0, 120.0, 270.0);
    public final Pose collectionPose = p.of(48.0, 122.0, 90.0);
    public final Pose parkPose = p.of(15.0, 120.0, 180.0);

    public final Path initialApproach;
    public final Path driveToFlower;
    public final Path flowerToScoring;
    public final Path scoringToPark;

    /** Constructs the four estimated strategy segments. */
    public RedScoringPaths() {
        initialApproach = straight(startPose, scoringPose);
        driveToFlower = fixHeadingStraight(scoringPose, collectionPose);
        flowerToScoring = fixHeadingStraight(collectionPose, scoringPose);
        scoringToPark = straight(scoringPose, parkPose);
    }

    /**
     * Constructs a line with heading interpolation.
     * @param from start pose
     * @param to destination pose
     * @return Pedro path
     */
    private static Path straight(Pose from, Pose to) {
        return Paths.line(from, to).linear(from, to);
    }

    /**
     * Builds a direct emergency route from the current measured pose.
     * IMPORTANT: Check obstacles and clearance before running this route.
     * @param currentPose current measured robot pose
     * @return emergency route
     */
    public Path emergencyPark(Pose currentPose) {
        return straight(currentPose, parkPose);
    }

   // public Path emergencyPark(Pose currentPose){
   //     if (currentPose.x()<30 && currentPose.y()>30){
   //         return straight(currentPose,parkPose);
   //     }
   //     return curve(currentPose,controlLegPose,parkPose);
   // }
    public static Path fixHeadingStraight(Pose from, Pose to){
        return Paths.line(from,to).constant(to.heading());
    }

    /**
     * Constructs a replacement path for an interrupted task (recovery path)
     * @param taskName  navigation task name
     * @param currentPose current robot pose
     * @param targetPose expected destination
     *
     * @return replacement path, or null if no safe route exists
     */
    public Path recoverPath(String taskName, Pose currentPose, Pose targetPose){
        switch(taskName){
            case "Initial Approach Path":
                return fixHeadingStraight(currentPose,scoringPose);
            case "Drive to Flower":
                return fixHeadingStraight(currentPose,collectionPose);
            case "Flower to Scoring":
                return fixHeadingStraight(currentPose,scoringPose);
            case "Drive to Park":
                return fixHeadingStraight(currentPose,parkPose);
            case "Emergency Park":
                return fixHeadingStraight(currentPose,parkPose);
            default:
                return null;

        }
    }
}

