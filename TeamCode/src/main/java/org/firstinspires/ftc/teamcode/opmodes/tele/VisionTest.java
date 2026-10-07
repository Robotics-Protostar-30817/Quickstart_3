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
    private final static Vision.Cell.Color focus= Vision.Cell.Color.RED;
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
        Vision.Cell cell = Robot.INSTANCE.vision.getBestCell(focus);
        if (cell!=null){
            telemetry.addLine("== CELL ==");
            telemetry.addData("Color",cell.getColor());
            telemetry.addData("Location", cell.getLocation());
            telemetry.addData("Scorable",cell.isScorable()?"YES":"NO");
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                    cell.getX(), cell.getY(), cell.getZ()));
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
