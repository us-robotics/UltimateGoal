package org.firstinspires.ftc.teamcode.Iteration4;

import org.firstinspires.ftc.teamcode.Behaviors.Drivetrain;
import org.firstinspires.ftc.teamcode.Behaviors.Intake;
import org.firstinspires.ftc.teamcode.Behaviors.Launcher;
import org.firstinspires.ftc.teamcode.Behaviors.WobbleGrabber;

import FTCEngine.Core.OpModeBase;
import FTCEngine.Math.Vector2;

public class SequenceB extends CommonSequence
{
	public SequenceB(OpModeBase opMode)
	{
		super(opMode);
	}

	@Override
	protected void queueJobs() {
		Drivetrain drivetrain = opMode.getBehavior(Drivetrain.class);
		WobbleGrabber wobbleGrabber = opMode.getBehavior(WobbleGrabber.class);
		Launcher launcher = opMode.getBehavior(Launcher.class);
		Intake intake = opMode.getBehavior(Intake.class);

		grabWobble(10f, 9f); //ends with wobble high

		//drive to square, let go, reset on wall
		execute(drivetrain, new Drivetrain.Move(new Vector2(0f, 100f)));
		execute(wobbleGrabber, new WobbleGrabber.Move(WobbleGrabber.Position.GRAB));
		execute(wobbleGrabber, new WobbleGrabber.Grab(false));
		execute(wobbleGrabber, new WobbleGrabber.Move(WobbleGrabber.Position.FOLD));
		execute(drivetrain, new Drivetrain.Drive(new Vector2(-1f, 0f), .5f));
		wait(.4f);

		execute(drivetrain, new Drivetrain.Move(new Vector2(8f, 0)));
		execute(drivetrain, new Drivetrain.Move(new Vector2(0f, -90f)));
		execute(drivetrain, new Drivetrain.Drive(new Vector2(0f, -1f), .5f));
		wait(.4f);

		grabWobble(4f, 9f);

		execute(drivetrain, new Drivetrain.Move(new Vector2(0f, 76f)));
		execute(drivetrain, new Drivetrain.Rotate(90f));
		execute(drivetrain, new Drivetrain.Move(new Vector2(-16f, 0f)));
		execute(drivetrain, new Drivetrain.Rotate(90f));
		execute(drivetrain, new Drivetrain.Drive(new Vector2(1f, 0f)));
	}
}