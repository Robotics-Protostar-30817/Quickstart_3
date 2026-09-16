package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static Follower create(HardwareMap h) {
        // return new Follower(Drivetrain, Localizer, Foresight);
        return null;
    }

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("leftFront");
        c.frontRightName.set("rightFront");
        c.backLeftName.set("leftBack");
        c.backRightName.set("rightBack");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(true);
    });

    /*
    public static MecanumConfig driveConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("leftFront");
                c.backLeftName.set("leftBack");
                c.frontRightName.set("rightFront");
                c.backRightName.set("rightBack");

                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

                c.manualBrakeMode.set(true);
            }
    );*/

    /*
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("odom");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(70.0);//measured 70 mm
        c.yPodOffset.set(100.0);//measured 100 mm
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);//tuner
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.MM);
        c.offsetUnits.set(DistanceUnit.MM);
    });
    */
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("odom");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
       // c.xPodOffset.set(-3.0451553825318345);
       // c.yPodOffset.set(3.831858297032634);
        //c.xPodOffset.set(-2.5956446730245757);
        //c.yPodOffset.set(4.142087200495202);
        //c.xPodOffset.set(-2.82040002777820);//average of two measurements
        c.xPodOffset.set(2.89);//manually measured.
        c.yPodOffset.set(3.986972748763918);
        //c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
}