package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Odom;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

/**
 * Main robot container class implementing the singleton pattern.
 * Provides centralized access to all robot subsystems including the drivetrain,
 * odometry, vision processing, and shooter.
 */
public class Robot {

    /**
     * Singleton instance of the robot container.
     */
    public static final Robot INSTANCE = new Robot();

    /**
     * Drivetrain subsystem instance for robot motion and motor control.
     */
    public final Drivetrain dt;

    /**
     * Odometry subsystem instance for robot tracking and localization.
     */
    public final Odom odom;

    /**
     * Vision subsystem instance for AprilTag target tracking and cell detection.
     */
    public final Vision vision;

    /**
     * Shooter subsystem instance for flywheel launching mechanism.
     */
    public final Shooter shooter;

    /**
     * Private constructor initializing all subsystem instances from their respective singletons.
     */
    private Robot() {
        dt = Drivetrain.INSTANCE;
        odom = Odom.INSTANCE;
        vision = Vision.INSTANCE;
        shooter = Shooter.INSTANCE;
    }
}
