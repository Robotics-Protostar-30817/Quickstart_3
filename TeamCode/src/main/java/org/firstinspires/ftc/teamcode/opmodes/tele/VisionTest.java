package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name="VisionTest",group="Test")
public class VisionTest extends OpMode {
    @Override
    public void init(){
        Robot.INSTANCE.vision.init(hardwareMap, telemetry);
        telemetry.addLine("Vision initialized");
        telemetry.addLine("Press Start and point Webcam to an AprilTag");
        telemetry.update();
    }

    @Override
    public void loop(){
        Robot.INSTANCE.vision.periodic();
    }

    @Override
    public void stop(){
        Robot.INSTANCE.vision.close();
    }
}
