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

		execute(drivetrain, new Drivetrain.Move(new Vector2(0f, 64f)));
		execute(wobbleGrabber, new WobbleGrabber.Move(WobbleGrabber.Mode.GRAB));

		execute(wobbleGrabber, new WobbleGrabber.Move(WobbleGrabber.Mode.IDLE));
		execute(drivetrain, new Drivetrain.Rotate(90f));
		execute(drivetrain, new Drivetrain.Rotate(90f));

		execute(wobbleGrabber, new WobbleGrabber.Grab(false)); //Drop wobble
		wait(0.5f);

		execute(drivetrain, new Drivetrain.Move(new Vector2(-8f, 0f)));
		execute(drivetrain, new Drivetrain.Move(new Vector2(0f, 14f)));
		execute(wobbleGrabber, new WobbleGrabber.Grab(true));

		execute(wobbleGrabber, new WobbleGrabber.Move(WobbleGrabber.Mode.FOLD));
		execute(drivetrain, new Drivetrain.Drive(Vector2.right, 0.7f));

		wait(1f);

		execute(drivetrain, new Drivetrain.Reset());
		execute(drivetrain, new Drivetrain.Drive(Vector2.zero));

		powerShots();
		linePark();
	}
}
