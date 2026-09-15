package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous
public class ExampleAuto extends OpMode {

    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();

    private final Pose StartPose=p.of(24,24,0);
    private final Pose park = p.of(48,48,90);

    private final Pose controlPose = p.of(36,60,45);
    private Path park(){
        return line(StartPose,park).linear(StartPose,park);
    }

    private Path parkOne(){
        return curve(StartPose,controlPose,park).linear(StartPose,park);
    }

    @Override
    public void init(){
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(StartPose);
        follower.update();
    }

    @Override
    public void start(){
        schedule(follow(follower,park()));
    }

    @Override
    public void loop(){
        follower.update();
        Scheduler.execute();
    }
}
