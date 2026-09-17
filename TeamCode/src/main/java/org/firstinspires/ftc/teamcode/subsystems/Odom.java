package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import dev.nextftc.core.subsystems.Subsystem;

public class Odom implements Subsystem {

    public static final Odom INSTANCE = new Odom();

    private GoBildaPinpointDriver pinpoint;
    private Telemetry telemetry;

    private Odom() {
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {

        this.telemetry = telemetry;

        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class,
                "odom"
        );

        pinpoint.setOffsets(70, 100, DistanceUnit.MM);

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
    }

    @Override
    public void periodic() {

        if (pinpoint != null) {

            pinpoint.update();

            telemetry.addData(
                    "X",
                    pinpoint.getPosX(DistanceUnit.MM)
            );

            telemetry.addData(
                    "Y",
                    pinpoint.getPosY(DistanceUnit.MM)
            );

            telemetry.addData(
                    "Heading",
                    pinpoint.getHeading(AngleUnit.DEGREES)
            );

            telemetry.update();
        }
    }

    public GoBildaPinpointDriver getPinpoint() {
        return pinpoint;
    }

    public void resetPosition() {

        pinpoint.setPosition(
                new Pose2D(
                        DistanceUnit.MM,
                        0,
                        0,
                        AngleUnit.DEGREES,
                        0
                )
        );
    }
}