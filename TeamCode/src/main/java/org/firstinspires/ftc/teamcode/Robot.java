package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Odom;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

public class Robot {

    public static final Robot INSTANCE = new Robot();

    public final Drivetrain dt;
    public final Odom odom;

    public final Vision vision;
    private Robot() {
        dt = Drivetrain.INSTANCE;
        odom = Odom.INSTANCE;
        vision = Vision.INSTANCE;
    }
}