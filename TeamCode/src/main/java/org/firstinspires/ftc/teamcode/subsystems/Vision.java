package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import dev.nextftc.core.subsystems.Subsystem;
public class Vision implements Subsystem{

    public static final Vision INSTANCE = new Vision();
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private Telemetry telemetry;

    //final camera position on robot
    //+x= robot right, +y = robot forward, +z = robot up, should be updated later
    private final Position cameraPosition  = new Position(DistanceUnit.INCH,-4,0,0,0);
    //camera is mounted to 4 incles left of the robot center, facing forward, no twist, no vertical offset.
    private final YawPitchRollAngles cameraAngles = new YawPitchRollAngles(AngleUnit.DEGREES,
            0,-45,0,0);//tilted upward 45 degree
    private Vision(){

    }
    public void init(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        aprilTag = new AprilTagProcessor.Builder()
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                    .setCameraPose(cameraPosition,cameraAngles)
                    .build();
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam1");
        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(aprilTag)
                .build();
    }

    @Override
    public void periodic(){
        if (aprilTag != null){
            List<AprilTagDetection> detections = aprilTag.getDetections();
            telemetry.addData("AprilTags Detected",detections.size());
            for (AprilTagDetection detection: detections){
                if (detection instanceof AprilTagClusterDetection){
                    AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;
                    VectorF fieldPos = clusterDet.metadata.fieldPosition;
                    Quaternion q = clusterDet.metadata.fieldOrientation;
                    Orientation angles = q.toOrientation(AxesReference.INTRINSIC,
                            AxesOrder.YXZ,AngleUnit.DEGREES);
                    DistanceUnit distanceU = clusterDet.metadata.distanceUnit;

                    telemetry.addLine(String.format("\n==== Tag Cluster (%s)", clusterDet.metadata.name));
                    telemetry.addLine(String.format("Percent tags found: %d", clusterDet.percentClusterFound));
                    telemetry.addLine(String.format("Cluster FieldPosition XYZ: %6.1f %6.1f %6.1f (%s)",
                            fieldPos.get(0),fieldPos.get(1),fieldPos.get(2), distanceU));
                    telemetry.addLine(String.format("Cluster Orientation PRY %6.1f %6.1f %6.1f (deg)",
                            angles.firstAngle,angles.secondAngle,angles.thirdAngle));

                    telemetry.addLine("Cluster's pos relative to the camera in FTC driver-centric metrics");
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                            detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                            detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)",
                            detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

                    telemetry.addLine("Robot's center relative to the field origin by Camera offset");
                    telemetry.addLine(String.format("Robot XYZ %6.1f %6.1f %6.1f (inch)",
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getPosition().z));
                    telemetry.addLine(String.format("Robot PRY %6.1f %6.1f %6.1f (deg)",
                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));

                }
            }
            // Add "key" information to telemetry
            telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
            telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
            telemetry.addLine("RBE = Range, Bearing & Elevation");
        }

        telemetry.update();
    }

    public List<AprilTagDetection> getDetections(){
        if (aprilTag == null){
            return java.util.Collections.emptyList();
        }
        return aprilTag.getDetections();
    }

    public VisionPortal getVisionPortal(){
        return visionPortal;
    }

    public AprilTagProcessor getAprilTagProcessor(){
        return aprilTag;
    }

    public void close(){
        if (visionPortal !=null){
            visionPortal.close();
        }
    }
}
