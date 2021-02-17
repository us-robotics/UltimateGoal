package org.firstinspires.ftc.teamcode.Iteration4;

import org.firstinspires.ftc.teamcode.Behaviors.DistanceSensors;
import org.firstinspires.ftc.teamcode.Behaviors.Drivetrain;
import org.firstinspires.ftc.teamcode.Behaviors.Launcher;
import org.firstinspires.ftc.teamcode.Behaviors.WobbleGrabber;

import FTCEngine.Core.Auto.JobSequence;
import FTCEngine.Core.OpModeBase;
import FTCEngine.Math.Vector2;

public abstract class CommonSequence extends JobSequence
{
	public CommonSequence(OpModeBase opMode)
	{
		super(opMode);
	}

	protected void grabWobble(float leftDistance, float rightDistance)
	{
		Drivetrain drivetrain = opMode.getBehavior(Drivetrain.class);
		WobbleGrabber wobbleGrabber = opMode.getBehavior(WobbleGrabber.class);

		buffer(drivetrain, new Drivetrain.Move(new Vector2(-leftDistance, 0f), 0.75f));
		buffer(wobbleGrabber, new WobbleGrabber.Move(WobbleGrabber.Mode.GRAB));

		execute();

		execute(wobbleGrabber, new WobbleGrabber.Grab(false));
		wait(0.5f);

		execute(drivetrain, new Drivetrain.Move(new Vector2(rightDistance, 0f), 0.45f));
		execute(wobbleGrabber, new WobbleGrabber.Grab(true));
		wait(0.5f);

		buffer(wobbleGrabber, new WobbleGrabber.Move(WobbleGrabber.Mode.HIGH));
		buffer(drivetrain, new Drivetrain.Move(new Vector2(leftDistance - rightDistance, 0f), 0.75f));

		execute();
	}

	protected void powerShots()
	{
		Drivetrain drivetrain = opMode.getBehavior(Drivetrain.class);
		Launcher launcher = opMode.getBehavior(Launcher.class);

		execute(launcher, new Launcher.Prime(Launcher.SHOT_POWER, true));

		execute(launcher, new Launcher.Launch(false));
		execute(drivetrain, new Drivetrain.Obstacle(56f));
		execute(drivetrain, new Drivetrain.Line());

		execute(drivetrain, new Drivetrain.Move(new Vector2(0f, 4f), 0.75f));

		//Launch first ring
		execute(launcher, new Launcher.Launch(true));
		wait(0.75f);

		execute(launcher, new Launcher.Launch(false));
		execute(drivetrain, new Drivetrain.Obstacle(72f));
		execute(drivetrain, new Drivetrain.Line());

		execute(drivetrain, new Drivetrain.Move(new Vector2(0f, 4f), 0.75f));

		//Launch second ring
		execute(launcher, new Launcher.Launch(true));
		wait(0.75f);

		execute(launcher, new Launcher.Launch(false));
		execute(drivetrain, new Drivetrain.Obstacle(88f));
		execute(drivetrain, new Drivetrain.Line());

		execute(drivetrain, new Drivetrain.Move(new Vector2(0f, 4f), 0.75f));

		//Launch third ring
		execute(launcher, new Launcher.Launch(true));
		wait(0.75f);

		execute(launcher, new Launcher.Launch(false));
		execute(launcher, new Launcher.Prime(Launcher.HIGH_POWER, false));
	}

	protected void grabWobble2()
	{
		Drivetrain drivetrain = opMode.getBehavior(Drivetrain.class);
		WobbleGrabber wobbleGrabber = opMode.getBehavior(WobbleGrabber.class);

		buffer(wobbleGrabber, new WobbleGrabber.Move(WobbleGrabber.Mode.FOLD));
		execute(drivetrain, new Drivetrain.Obstacle(130f));

		execute(drivetrain, new Drivetrain.Line());

		buffer(wobbleGrabber, new WobbleGrabber.Move(WobbleGrabber.Mode.GRAB));
		execute(drivetrain, new Drivetrain.Move(new Vector2(0f, 30f)));

		execute(drivetrain, new Drivetrain.Obstacle(38f, DistanceSensors.Side.FRONT));

		execute(wobbleGrabber, new WobbleGrabber.Grab(false));
		execute(drivetrain, new Drivetrain.Obstacle(100f));

		execute(wobbleGrabber, new WobbleGrabber.Grab(true));
		execute(drivetrain, new Drivetrain.Move(new Vector2(0f, -30f)));

		buffer(wobbleGrabber, new WobbleGrabber.Move(WobbleGrabber.Mode.HIGH));
		execute(drivetrain, new Drivetrain.Move(new Vector2(0f, -30f)));
	}

	protected void linePark()
	{
		Drivetrain drivetrain = opMode.getBehavior(Drivetrain.class);

		execute(drivetrain, new Drivetrain.Line());
		execute(drivetrain, new Drivetrain.Move(new Vector2(0f, -12f), 0.65f));
	}
}
