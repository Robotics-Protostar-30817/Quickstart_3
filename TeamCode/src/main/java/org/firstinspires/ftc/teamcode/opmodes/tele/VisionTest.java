package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

@TeleOp(name="VisionTest",group="Test")
/**
 * 
 */
public class VisionTest extends OpMode {
    @Override
    /**
     * 
     */
    public void init(){
        Robot.INSTANCE.vision.init(hardwareMap, telemetry);
        telemetry.addLine("Vision initialized");
        telemetry.addLine("Press Start and point Webcam to a Cell");
        telemetry.update();
    }

    @Override
    /**
     * 
     */
    public void loop(){
        //vision.periodic() displays the actual cluster pose returned by the SDK.
        Robot.INSTANCE.vision.periodic();
        Vision.Cell cell = Robot.INSTANCE.vision.getBestCell();
        if (cell!=null){
            telemetry.addLine("== CELL ==");
            telemetry.addData("Name", cell.getName());
            telemetry.addData("Color",cell.getColor());
            telemetry.addData("Location", cell.getLocation());
            telemetry.addData("Scorable",cell.isScorable()?"YES":"NO");
            telemetry.addData("Range","%.1f in",cell.getRange());
            telemetry.addData("Bearing","%.1f deg",cell.getBearing());
            telemetry.addData("Elevation","%.1f deg",cell.getElevation());
            telemetry.addData("Is Red", cell.isRed());
            telemetry.addData("Is Scoring", cell.isScoring());

        }else{
            telemetry.addLine("No Cell detected");
        }
        telemetry.update();
    }

    @Override
    /**
     * 
     */
    public void stop(){
        Robot.INSTANCE.vision.close();
    }
}
