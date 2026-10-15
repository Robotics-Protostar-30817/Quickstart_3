package org.firstinspires.ftc.teamcode.pedro.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Field poses and six Pedro Pathing 3 routes from the REDAUDIENCE strategy sheet.
 *
 * <p>Coordinates are in inches in the global 144 x 144 Pedro field frame.
 * Headings are radians (0 = +X, 90 degrees = +Y).
 *
 * <p>The strategy sheet specifies XY waypoints but not robot headings. The
 * headings below are initial engineering choices and must be validated with
 * the camera's Red Audience CELL visibility and the intake approach direction.
 *
 * <p>All routes are straight lines. Check clearance with the full 16 x 16 inch
 * robot footprint before operating; the strategy diagram is not a collision
 * certificate.
 */
public final class RedAudiencePaths {
    private final PoseFactory p = PoseFactory.degrees();

    /** Initial robot-center pose, from the strategy sheet. */
    public final Pose startPose = p.of(60.0, 10.0, 90.0);

    /** Common shooting pose for the Red Audience CELL. */
    public final Pose shootPose = p.of(60.0, 30.0, 90.0);

    /** Garden pickup pose near the lower-left corner. */
    public final Pose gardenPose = p.of(17.0, 14.0, 269.8);

    /** Flower pickup pose along the left side of the field. */
    public final Pose flowerPose = p.of(17, 48.0, 180.0) ;

    /** Strategy-sheet parking target. */
    public final Pose parkPose = p.of(17.0, 97.0, 135.0);

    public final Pose controlLegPose = p.of(20,30,0);
    public final Path initialApproach;
    public final Path driveToGarden;
    public final Path gardenToAudience;
    public final Path driveToFlower;
    public final Path flowerToAudience;
    public final Path driveToPark;

    /** Builds the six routes once, before autonomous begins. */
    public RedAudiencePaths() {
        initialApproach = straight(startPose, shootPose);
        //driveToGarden = straight(shootPose, gardenPose);
        driveToGarden = fixHeadingStraight(shootPose,gardenPose);
        //gardenToAudience = straight(gardenPose, shootPose);
        gardenToAudience = fixHeadingStraight(gardenPose,shootPose);
        //driveToFlower = straight(shootPose, flowerPose);
        driveToFlower = fixHeadingStraight(shootPose,flowerPose);
        ///flowerToAudience = straight(flowerPose, shootPose);
        flowerToAudience = fixHeadingStraight(flowerPose,shootPose);
        //driveToPark = straight(shootPose, parkPose);
        driveToPark = curve(shootPose,controlLegPose,parkPose);
    }


    /**
     * Creates a straight segment and interpolates heading during travel.
     *
     * @param from start pose
     * @param to end pose
     * @return configured Pedro path
     */
    private static Path straight(Pose from, Pose to) {
        return Paths.line(from, to).linear(from, to);
    }

    private static Path fixHeadingStraight(Pose from, Pose to){
        return Paths.line(from,to).constant(to.heading());
    }

    private static Path curve(Pose from, Pose control, Pose to){
        return Paths.curve(from,control,to).tangent();
    }
    /**
     * Creates an emergency parking route from the robot's CURRENT pose.
     * This avoids assuming the robot is already at the shooting waypoint.
     *
     * @param currentPose current pose reported by Pedro localization
     * @return direct path to the parking pose
     */
    //public Path emergencyPark(Pose currentPose) {
    //    return straight(currentPose, parkPose);
    //}
    public Path emergencyPark(Pose currentPose){
        if (currentPose.x()<30 && currentPose.y()>30){
            return straight(currentPose,parkPose);
        }
        return curve(currentPose,controlLegPose,parkPose);
    }
}

