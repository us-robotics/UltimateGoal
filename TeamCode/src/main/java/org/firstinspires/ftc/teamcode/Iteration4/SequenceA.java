package org.firstinspires.ftc.teamcode.Iteration4;

import org.firstinspires.ftc.teamcode.Behaviors.Drivetrain;
import org.firstinspires.ftc.teamcode.Behaviors.WobbleGrabber;

import FTCEngine.Core.OpModeBase;
import FTCEngine.Math.Vector2;

public class SequenceA extends CommonSequence
{
	public SequenceA(OpModeBase opMode)
	{
		super(opMode);
	}

	@Override
	protected void queueJobs()
	{
		Drivetrain drivetrain = opMode.getBehavior(Drivetrain.class);
		WobbleGrabber wobbleGrabber = opMode.getBehavior(WobbleGrabber.class);

		execute(wobbleGrabber, new WobbleGrabber.Move(WobbleGrabber.Mode.GRAB));
		execute(drivetrain, new Drivetrain.Move(new Vector2(0f, 62f)));

		execute(wobbleGrabber, new WobbleGrabber.Move(WobbleGrabber.Mode.IDLE));
		execute(drivetrain, new Drivetrain.Rotate(135f));

		execute(wobbleGrabber, new WobbleGrabber.Grab(false)); //Drop wobble
		wait(0.5f);

		execute(drivetrain, new Drivetrain.Move(new Vector2(-15f, 0f)));
		execute(wobbleGrabber, new WobbleGrabber.Grab(true));

		execute(wobbleGrabber, new WobbleGrabber.Move(WobbleGrabber.Mode.FOLD));

		execute(drivetrain, new Drivetrain.Rotate(45f));
		execute(drivetrain, new Drivetrain.Obstacle(10f));

		execute(drivetrain, new Drivetrain.Move(new Vector2(0f, -4f)));
		execute(drivetrain, new Drivetrain.Drive(Vector2.right, 0.7f));

		wait(1f);

		execute(drivetrain, new Drivetrain.Reset());
		execute(drivetrain, new Drivetrain.Drive(Vector2.zero));

		powerShots();
		execute(drivetrain, new Drivetrain.Line());
	}
}
