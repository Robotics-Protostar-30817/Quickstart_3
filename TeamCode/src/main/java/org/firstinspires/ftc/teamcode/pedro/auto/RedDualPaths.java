package org.firstinspires.ftc.teamcode.pedro.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public final class RedDualPaths {
    private final PoseFactory p = PoseFactory.degrees();
    public final Pose startPose = p.of(60.0, 10.0, 90.0);
    public final Pose shootAudiencePose = p.of(60.0, 30.0, 90.0);

    public final Pose gardenAudiencePose = p.of(17.0, 14.0, 269.8);
    public final Pose flowerAudiencePose = p.of(17, 48.0, 180.0) ;

    public final Pose shootScoringPose = p.of(60.0, 120.0, 270.0);

    public final Pose flowerScoringPose = p.of(48.0, 122.0, 90.0);
    public final Pose parkPose = p.of(10.0, 108.0, 135.0);
    public final Pose controlLegPose = p.of(12,108,0);

    public final Path initalApproach;
    public final Path driveToAudienceFlower;
    public final Path audienceFlowerToScoring;
    public final Path driveToSocringFlower;
    public final Path scoringFlowerToScoring;
    public final Path scoringDriveToPark;

        public RedDualPaths() {
            initalApproach = straight(startPose, shootAudiencePose);
            driveToAudienceFlower = fixHeadingStraight(shootAudiencePose, gardenAudiencePose);
            audienceFlowerToScoring = curve(gardenAudiencePose, controlLegPose, shootScoringPose);
            driveToSocringFlower = fixHeadingStraight(shootScoringPose, flowerScoringPose);
            scoringFlowerToScoring = fixHeadingStraight(flowerScoringPose, shootScoringPose);
            scoringDriveToPark = straight(shootScoringPose, parkPose);
        }

        private static Path straight(Pose from, Pose to) {
            return Paths.line(from, to).linear(from, to);
        }

        private static Path fixHeadingStraight(Pose from, Pose to){
            return Paths.line(from,to).constant(to.heading());
        }

        private static Path curve(Pose from, Pose control, Pose to){
            return Paths.curve(from,control,to).tangent();
        }

        private Path emergencyPark(Pose currentPose){
                return straight(currentPose,parkPose);
        }
    }
