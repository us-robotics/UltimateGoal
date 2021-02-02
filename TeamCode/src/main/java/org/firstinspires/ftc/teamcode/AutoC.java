package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Behaviors.*;

import java.util.List;

import FTCEngine.Core.Auto.ConfigOption;
import FTCEngine.Core.Auto.JobSequence;
import FTCEngine.Core.Behavior;
import FTCEngine.Core.OpModeBase;
import FTCEngine.Math.Vector2;

@Autonomous(name = "AutoC")
public class AutoC extends OpModeBase
{
    @Override
    protected void addBehaviors(List<Behavior> behaviorList) {
        behaviorList.add(new Drivetrain(this));
        behaviorList.add(new WobbleGrabberOldAuto(this));
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
       assignSequence(new AutoSequenceC(this));
    }

    public static class AutoSequenceC extends JobSequence
    {
        public AutoSequenceC(OpModeBase opMode) {
            super(opMode);
        }

        @Override
        protected void queueJobs() {

            Drivetrain drivetrain = opMode.getBehavior(Drivetrain.class);
            WobbleGrabber wobbleGrabber = opMode.getBehavior(WobbleGrabber.class);
            Launcher launcher = opMode.getBehavior(Launcher.class);
            Intake intake = opMode.getBehavior(Intake.class);

            grabWobble(10f, 9f);

            execute(drivetrain, new Drivetrain.Move(new Vector2(0f, 104f)));

            execute(drivetrain, new Drivetrain.Move(new Vector2(20f, 0f)));
            execute(drivetrain, new Drivetrain.Rotate(-90f));
            execute(drivetrain, new Drivetrain.Rotate(-90f));

            execute(wobbleGrabber, new WobbleGrabber.Move(WobbleGrabber.Position.GRAB));
            wait(0.4f);

            //Hit wall to Reset Y
            execute(drivetrain, new Drivetrain.Move(new Vector2(0f, -12f)));
            execute(drivetrain, new Drivetrain.Drive(new Vector2(0f, -1f), 0.5f));
            wait(0.7f);

            execute(drivetrain, new Drivetrain.Drive(Vector2.zero));
            execute(drivetrain, new Drivetrain.Reset());
            execute(wobbleGrabber, new WobbleGrabber.Grab( true));

            execute(launcher, new Launcher.Prime(Launcher.HIGH_POWER - 0.02f, true));
            execute(drivetrain, new Drivetrain.Move(new Vector2(-10f, 0f)));
            execute(wobbleGrabber, new WobbleGrabber.Move(WobbleGrabber.Position.FOLD));

            //Forward to launch position
            execute(drivetrain, new Drivetrain.Move(new Vector2(0f, 61f)));
            wait(0.5f);

            execute(drivetrain, new Drivetrain.Move(new Vector2(24f, 0f)));
            execute(drivetrain, new Drivetrain.Drive(new Vector2(1f, 0f), 0.55f));

            wait(0.7f);
            execute(drivetrain, new Drivetrain.Drive(Vector2.zero));
            execute(drivetrain, new Drivetrain.Reset());

            //Launch ring 1
            execute(launcher, new Launcher.Launch(true));
            wait(1f);

            execute(launcher, new Launcher.Launch(false));
            execute(launcher, new Launcher.Prime(Launcher.HIGH_POWER, true));
            wait(1f);

            //Launch ring 2
            execute(launcher, new Launcher.Launch(true));
            wait(1f);

            execute(launcher, new Launcher.Launch(false));
            execute(launcher, new Launcher.Prime(Launcher.HIGH_POWER, true));

            execute(intake, new Intake.Run(0.4f));
            wait(0.5f);
            execute(intake, new Intake.Run(0f));
            wait(0.5f);

            //Launch ring 3
            execute(launcher, new Launcher.Launch(true));
            wait(1f);

            execute(launcher, new Launcher.Launch(false));
            execute(launcher, new Launcher.Prime(0f, false));

            //Rotate back to starting rotation
            execute(drivetrain, new Drivetrain.Move(new Vector2(-17f, 0f)));
            execute(drivetrain, new Drivetrain.Move(new Vector2(0f, 44f)));
            execute(drivetrain, new Drivetrain.Rotate(180f));

            //Hit wall reset Y
            execute(drivetrain, new Drivetrain.Drive(new Vector2(0f, -1f), 0.55f));

            wait(0.75f);
            execute(drivetrain, new Drivetrain.Drive(Vector2.zero));
            execute(drivetrain, new Drivetrain.Reset());

            execute(drivetrain, new Drivetrain.Move(new Vector2(8f, 0f), 0.55f));
            execute(drivetrain, new Drivetrain.Move(new Vector2(0f, 13f), 0.55f));

            grabWobble(4f, 9f);

            execute(drivetrain, new Drivetrain.Move(new Vector2(-6f, 0f)));
            execute(drivetrain, new Drivetrain.Move(new Vector2(0f, 108f)));

            execute(wobbleGrabber, new WobbleGrabber.Move(WobbleGrabber.Position.FOLD));
            wait(0.4f);

            wait(0.5f);

            execute(wobbleGrabber, new WobbleGrabber.Grab(true));
            wait(0.5f);

            execute(drivetrain, new Drivetrain.Move(new Vector2(-10f, 0f)));
            execute(wobbleGrabber, new WobbleGrabber.Move(WobbleGrabber.Position.FOLD));
            execute(drivetrain, new Drivetrain.Move(new Vector2(0f, -36f)));
        }

        private void grabWobble(float leftDistance, float rightDistance)
        {
            Drivetrain drivetrain = opMode.getBehavior(Drivetrain.class);
            WobbleGrabber wobbleGrabber = opMode.getBehavior(WobbleGrabber.class);

            execute(drivetrain, new Drivetrain.Move(new Vector2(-leftDistance, 0f), 0.75f));
            execute(wobbleGrabber, new WobbleGrabber.Move(WobbleGrabber.Position.GRAB));

            execute(wobbleGrabber, new WobbleGrabber.Grab(true));
            execute(drivetrain, new Drivetrain.Move(new Vector2(rightDistance, 0f), 0.55f));

            wait(0.5f);

            wait(0.3f);

            execute(drivetrain, new Drivetrain.Move(new Vector2(-4f, 0f), 0.75f));
            execute(wobbleGrabber, new WobbleGrabber.Move(WobbleGrabber.Position.GRAB));
        }
    }
}
