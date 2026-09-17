package org.firstinspires.ftc.teamcode.subsystems;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class Drivetrain implements Subsystem {


    public static final Drivetrain INSTANCE = new Drivetrain();

    // MOTOR CONFIGURATION NAMES
    // These strings are the names that we will give the motors in the FTC Robot Controller configuration.
    // IMPORTANT: The names here MUST exactly match the names configured on the robot.
    private final String rightFrontName = "rightFront";
    private final String leftFrontName = "leftFront";
    private final String rightBackName = "rightBack";
    private final String leftBackName = "leftBack";

    //MOTOR OBJECTS
    //These variables represent the actual motors.
    //Left side is reversed.
    private final MotorEx rightFrontMotor = new MotorEx(rightFrontName);

    private final MotorEx leftFrontMotor = new MotorEx(leftFrontName).reversed();

    private final MotorEx rightBackMotor = new MotorEx(rightBackName);

    private final MotorEx leftBackMotor = new MotorEx(leftBackName).reversed();


    private Drivetrain() {
        // The motors are already created above.
    }

    public MotorEx getLeftFrontMotor() {
        return leftFrontMotor;
    }

    public MotorEx getRightFrontMotor() {
        return rightFrontMotor;
    }

    public MotorEx getLeftBackMotor() {
        return leftBackMotor;
    }

    public MotorEx getRightBackMotor() {
        return rightBackMotor;
    }

}
