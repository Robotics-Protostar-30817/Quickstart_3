package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.auto.PathUtility;

@Autonomous(name = "Red Auto - State Machine", group = "Autonomous")
public final class RedDualAuto extends OpMode {
    private static final String LOG_TAG = "RedDualAuto";
    private enum RState implements PathUtility.AutoState {
        READY,
        AUDIENCE_TO_SCORING,
        SCORING_TO_AUDIENCE,
        FINISHED,
        FAULT
    }

    private static final String TAG="RedDualAuto";
    private static final double AUTO_SECONDS=30;
    private static final double PARK_RESERVE_SECOND=5.0;
    //for testing without motor shooting
    private static final double SHOOT_SECONDS=2;


    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}
