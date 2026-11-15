package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.subsystems.IntakeRamp;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

/**
 * Floor POLLEN collection test only. The upper feed motor is ALWAYS stopped,
 * the blocker is ALWAYS commanded closed, and the flywheel is not initialized.
 *
 * Configure these device names to match the Robot Configuration.
 * With no POLLEN, first verify motor and ramp directions at low power.
 */
@TeleOp(name = "POLLEN Collection Test", group = "Test")
public final class PollenCollectionTest extends OpMode {
    public static String LOWER_MOTOR_NAME = "transfer1";
    public static String UPPER_MOTOR_NAME = "transfer2";
    public static String RAMP_SERVO_NAME = "intake";
    public static String BLOCKER_SERVO_NAME = "blocker";

    // Start conservatively; change signs if the mechanisms move backward.
    public static double LOWER_INTAKE_POWER = 0.25;
    //public static double RAMP_INTAKE_POWER = 0.25;

    private final Transfer transfer = Transfer.INSTANCE;
    private final IntakeRamp ramp = IntakeRamp.INSTANCE;
    private final Blocker blocker = Blocker.INSTANCE;
    private boolean prevA;
    private boolean collecting;

    @Override public void init() {
        transfer.init(
                hardwareMap.get(DcMotorEx.class, LOWER_MOTOR_NAME),
                hardwareMap.get(DcMotorEx.class, UPPER_MOTOR_NAME));
        ramp.init(hardwareMap.get(Servo.class, RAMP_SERVO_NAME));
        blocker.init(hardwareMap.get(Servo.class, BLOCKER_SERVO_NAME));
        stopCollection();
        blocker.close();
        telemetry.addLine("A: toggle floor collection | B: stop");
        telemetry.addLine("Upper feed motor stays OFF; blocker stays CLOSED.");
        telemetry.addLine("Verify blocker CLOSED_POSITION before using POLLEN.");
        telemetry.update();
    }

    @Override public void loop() {
        boolean a = gamepad1.a;
        if (gamepad1.b) {
            stopCollection();
        } else if (a && !prevA) {
            if (collecting) stopCollection();
            else startCollection();
        }
        prevA = a;

        // Reassert safety constraints on every loop.
        transfer.stopFeed();
        blocker.close();
        transfer.periodic();
        ramp.periodic();
        blocker.periodic();

        telemetry.addData("Collecting", collecting);
        telemetry.addData("Lower intake power", "%.2f", transfer.getIntakePower());
        telemetry.addData("Lower motor velocity (ticks/s)", "%.1f", transfer.getVelocity1());
        telemetry.addData("Ramp position (commanded)", "%.2f", ramp.getPosition());
        telemetry.addData("Ramp lowered (commanded)", "%.1f", ramp.isLowered());
        telemetry.addData("Upper feed power (must be 0)", "%.2f", transfer.getFeedPower());
        telemetry.addData("Blocker closed (commanded)", !blocker.isOpen());
        telemetry.addLine("No sensor confirms the blocker physical position.");
        telemetry.addLine("A toggle collection | B stop");
        telemetry.update();
    }

    private void startCollection() {
        if (!Double.isFinite(LOWER_INTAKE_POWER) || Math.abs(LOWER_INTAKE_POWER) > 1.0) {
            stopCollection();
            telemetry.addLine("Invalid collection powers; collection not started");
            return;
        }
        blocker.close();
        transfer.stopFeed();
        transfer.setIntakePower(LOWER_INTAKE_POWER);
        ramp.lower();
        collecting = true;
    }

    private void stopCollection() {
        transfer.stopIntake();
        transfer.stopFeed();
        ramp.stop();
        collecting = false;
    }

    @Override public void stop() {
        stopCollection();
        blocker.close();
    }
}

