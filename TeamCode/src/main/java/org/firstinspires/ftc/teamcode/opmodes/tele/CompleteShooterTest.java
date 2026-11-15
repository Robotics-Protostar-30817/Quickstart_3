package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.subsystems.*;

/** Complete shooter integration test. Start without POLLEN; use a safe backstop.
 *  A: aim on (optional), B: aim off/cancel, X: RED, Y: BLUE
 *  Left bumper: toggle floor collection; right bumper: aim AND shoot continuously
 *  D-pad down: STOP continuous shooting; D-pad left: stop collection.
 */
@TeleOp(name = "Complete Shooter Integration Test", group = "Test")
public final class CompleteShooterTest extends OpMode {
    private final CompleteShooter shooter = CompleteShooter.INSTANCE;
    private final AutoAim aim = AutoAim.INSTANCE;
    private final Turret turret = Turret.INSTANCE;
    private final Hood hood = Hood.INSTANCE;
    private final Flywheel flywheel = Flywheel.INSTANCE;
    private final Transfer transfer = Transfer.INSTANCE;
    private final Blocker blocker = Blocker.INSTANCE;
    private final IntakeRamp ramp = IntakeRamp.INSTANCE;
    private final Vision vision = Vision.INSTANCE;
    private Vision.Cell.Color alliance = Vision.Cell.Color.RED;
    private boolean prevA, prevB, prevX, prevY, prevLB, prevRB, prevDown, prevLeft;
    private boolean rumbleSentForRequest;

    @Override public void init() {
        // Change names to match your robot configuration.
        turret.init(hardwareMap.get(DcMotorEx.class, "turret"));
        hood.init(hardwareMap.get(Servo.class, "hood"));
        flywheel.init(hardwareMap.get(DcMotorEx.class, "flywheel"));
        transfer.init(hardwareMap.get(DcMotorEx.class, "transferMotor1"),
                hardwareMap.get(DcMotorEx.class, "transferMotor2"));
        blocker.init(hardwareMap.get(Servo.class, "blocker"));
        ramp.init(hardwareMap.get(Servo.class, "intakeRamp"));
        vision.init(hardwareMap, telemetry); // Vision subsystem currently expects Webcam1.
        shooter.init(alliance);
        telemetry.addLine("A aim ON | B aim OFF | X RED | Y BLUE");
        telemetry.addLine("LB toggle collection | RB auto-aim + continuous shooting");
        telemetry.addLine("D-pad DOWN STOP shooting | LEFT stop collection");
        telemetry.addLine("FIRST TEST WITHOUT POLLEN. Check blocker CLOSED position.");
        telemetry.addLine("STOP robot before shooting: no motion compensation in AutoAim.");
        telemetry.update();
    }

    @Override public void loop() {
        if (gamepad1.b && !prevB) shooter.disableAim();
        if (gamepad1.x && !prevX) selectAlliance(Vision.Cell.Color.RED);
        if (gamepad1.y && !prevY) selectAlliance(Vision.Cell.Color.BLUE);
        if (gamepad1.a && !prevA) shooter.enableAim();
        if (gamepad1.left_bumper && !prevLB) {
            if (shooter.isCollecting()) shooter.stopCollecting();
            else shooter.startCollecting();
        }
        if (gamepad1.right_bumper && !prevRB) {
            if (!shooter.isShootingActive()) {
                rumbleSentForRequest = false;
                shooter.requestShot();
            }
        }
        if (gamepad1.dpad_down && !prevDown) {
            shooter.cancelShot();
            rumbleSentForRequest = false;
        }
        if (gamepad1.dpad_left && !prevLeft) shooter.stopCollecting();

        // Do not call aim.periodic() separately; shooter.periodic() owns it.
        shooter.periodic();
        boolean aimReadyNow = shooter.isAimReady();
        if (shooter.isShootingActive() && aimReadyNow && !rumbleSentForRequest) {
            gamepad1.rumble(300);
            rumbleSentForRequest = true;
        }
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Shot calibration verified", AutoAim.CALIBRATION_VERIFIED);
        telemetry.addData("Hood calibration valid", hood.hasValidAngleCalibration());
        telemetry.addData("Aim enabled", aim.isEnabled());
        telemetry.addData("Aim status", aim.getStatus());
        telemetry.addData("CELL distance (in)", aim.getDistanceInches());
        Vision.Cell cell = vision.getBestCell(alliance);
        telemetry.addData("CELL", cell == null ? "none" : cell.getName());
        telemetry.addData("Turret target (deg)", aim.getTargetTurretDegrees());
        telemetry.addData("Turret measured (deg)", turret.getAngleDegrees());
        telemetry.addData("Turret ready", turret.isAtTarget());
        telemetry.addData("Hood target launch (deg)", aim.getTargetHoodAngleDegrees());
        telemetry.addData("Hood target servo position", aim.getTargetHoodPosition());
        telemetry.addData("Hood commanded servo position", hood.getPosition());
        telemetry.addData("Hood time ready", hood.isReady());
        telemetry.addData("Flywheel target RPM", aim.getTargetFlywheelRPM());
        telemetry.addData("Flywheel measured RPM", flywheel.getFlywheelRPM());
        telemetry.addData("Flywheel speed ready", flywheel.isReadyToShoot());
        telemetry.addData("Aim ready", aimReadyNow);
        telemetry.addData("Collecting", shooter.isCollecting());
        telemetry.addData("Lower intake power", transfer.getIntakePower());
        telemetry.addData("Ramp lowered", ramp.isLowered());
        telemetry.addData("Ramp servo position", ramp.getPosition());
        telemetry.addData("Upper feed power", transfer.getFeedPower());
        telemetry.addData("Blocker open", blocker.isOpen());
        telemetry.addData("Shot pending", shooter.isShotPending());
        telemetry.addData("Continuous feeding", shooter.isFiring());
        telemetry.addData("Storage empty detection", "NOT AVAILABLE - operator must check");
        telemetry.addData("Max feed seconds", CompleteShooter.MAX_CONTINUOUS_FEED_SECONDS);
        telemetry.addData("Stage", shooter.getStage());
        telemetry.addData("Shooter status", shooter.getStatus());
        telemetry.update();

        prevA = gamepad1.a; prevB = gamepad1.b;
        prevX = gamepad1.x; prevY = gamepad1.y;
        prevLB = gamepad1.left_bumper; prevRB = gamepad1.right_bumper;
        prevDown = gamepad1.dpad_down; prevLeft = gamepad1.dpad_left;
    }

    private void selectAlliance(Vision.Cell.Color color) {
        shooter.stop();
        alliance = color;
        shooter.init(color);
        rumbleSentForRequest = false;
    }

    @Override public void stop() {
        shooter.stop();
        vision.close();
    }
}
