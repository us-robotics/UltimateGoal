package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Behaviors.Drivetrain;
import org.firstinspires.ftc.teamcode.Behaviors.InertialMeasurementUnit;
import org.firstinspires.ftc.teamcode.Behaviors.Intake;
import org.firstinspires.ftc.teamcode.Behaviors.Launcher;
import org.firstinspires.ftc.teamcode.Behaviors.WobbleGrabber;

import java.util.List;

import FTCEngine.Core.Auto.ConfigOption;
import FTCEngine.Core.Auto.JobSequence;
import FTCEngine.Core.Behavior;
import FTCEngine.Core.OpModeBase;
import FTCEngine.Math.Vector2;

@Autonomous(name = "AutoB")
public class AutoB extends OpModeBase {

    @Override
    protected void addBehaviors(List<Behavior> behaviorList) {
        behaviorList.add(new Drivetrain(this));
        behaviorList.add(new WobbleGrabber(this));
        behaviorList.add(new Launcher(this));
        behaviorList.add(new Intake(this));
        behaviorList.add(new InertialMeasurementUnit(this));
    }

    @Override
    protected void appendConfigOptions(List<ConfigOption> options) {

    }

    @Override
    public void init() {
        super.init();
        assignSequence(new AutoSequence(this));
    }

    @Override
    public void loop()
    {
        debug.addData("FPS", 1f / time.getDeltaTime());
        super.loop();
    }

    private static class AutoSequence extends JobSequence {
        public AutoSequence(OpModeBase opMode) {
            super(opMode);
        }

        @Override
        protected void queueJobs() {
            Drivetrain drivetrain = opMode.getBehavior(Drivetrain.class);
            WobbleGrabber wobbleGrabber = opMode.getBehavior(WobbleGrabber.class);
            Launcher launcher = opMode.getBehavior(Launcher.class);
            Intake intake = opMode.getBehavior(Intake.class);

            grabWobble(10f, 9f);

            //execute(drivetrain, new Drivetrain.Move(new Vector2(0f, 92f)));
            //execute();

        }

        private void grabWobble(float leftDistance, float rightDistance)
        {
            Drivetrain drivetrain = opMode.getBehavior(Drivetrain.class);
            WobbleGrabber wobbleGrabber = opMode.getBehavior(WobbleGrabber.class);

            execute(wobbleGrabber, new WobbleGrabber.Grab(false));
            execute(wobbleGrabber, new WobbleGrabber.Move(WobbleGrabber.Position.GRAB));

            execute(drivetrain, new Drivetrain.Move(new Vector2(-leftDistance, 0f), 0.75f));

            execute(drivetrain, new Drivetrain.Move(new Vector2(rightDistance, 0f), 0.55f));

            wait(0.3f);
            execute(wobbleGrabber, new WobbleGrabber.Grab(true));
            execute(wobbleGrabber, new WobbleGrabber.Move(WobbleGrabber.Position.HIGH));

            execute(drivetrain, new Drivetrain.Move(new Vector2(-4f, 0f), 0.75f));
        }
    }
}
