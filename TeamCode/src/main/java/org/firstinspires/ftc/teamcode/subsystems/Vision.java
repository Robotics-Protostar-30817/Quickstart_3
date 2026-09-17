package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import dev.nextftc.core.subsystems.Subsystem;
public class Vision implements Subsystem{

    public static final Vision INSTANCE = new Vision();
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private Telemetry telemetry;

    private Vision(){

    }
    public void init(HardwareMap hardwareMap, Telemetry telemetry){
            this.telemetry = telemetry;
            aprilTag = new AprilTagProcessor.Builder().build();
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam1");
        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(aprilTag)
                .build();
    }

    @Override
    public void periodic(){
        if (aprilTag !=null){
            List<AprilTagDetection> detections =
                    aprilTag.getDetections();
            telemetry.addData("AprilTags Detected",detections.size());
            for (AprilTagDetection detection: detections){
                telemetry.addData("Tag ID", detection.id);
            }
            telemetry.update();
        }
    }

    public boolean seesTag(int tagId){
        if (aprilTag == null){
            return false;
        }
        if (aprilTag.getDetections().size() == 0){
            return false;
        }
        for(AprilTagDetection detection: aprilTag.getDetections()){
            if (detection.id == tagId){
                return true;
            }
        }
        return false;
    }

    public List<AprilTagDetection> getDections(){
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
