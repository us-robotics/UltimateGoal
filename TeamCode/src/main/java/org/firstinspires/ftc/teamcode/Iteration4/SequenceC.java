package org.firstinspires.ftc.teamcode.Iteration4;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Behaviors.Drivetrain;
import org.firstinspires.ftc.teamcode.Behaviors.Intake;
import org.firstinspires.ftc.teamcode.Behaviors.Launcher;
import org.firstinspires.ftc.teamcode.Behaviors.WobbleGrabber;

import FTCEngine.Core.OpModeBase;
import FTCEngine.Math.Vector2;

public class SequenceC extends CommonSequence
{
	public SequenceC(OpModeBase opMode)
	{
		super(opMode);
	}

	@Override
	protected void queueJobs()
	{
		Drivetrain drivetrain = opMode.getBehavior(Drivetrain.class);
		WobbleGrabber wobbleGrabber = opMode.getBehavior(WobbleGrabber.class);
		Launcher launcher = opMode.getBehavior(Launcher.class);
		Intake intake = opMode.getBehavior(Intake.class);

		buffer(wobbleGrabber, new WobbleGrabber.Move(WobbleGrabber.Mode.GRAB));
		execute(drivetrain, new Drivetrain.Move(new Vector2(0f, 104f)));

		execute(drivetrain, new Drivetrain.Rotate(-90f));
		execute(drivetrain, new Drivetrain.Rotate(-90f));

		//Hit wall reset Y
		execute(drivetrain, new Drivetrain.Move(new Vector2(0f,-12f)));
		execute(drivetrain, new Drivetrain.Drive(new Vector2(0f, -1f), 0.7f));

		wait(0.5f);

		execute(drivetrain, new Drivetrain.Reset());
		execute(drivetrain, new Drivetrain.Drive(Vector2.zero));

		execute(wobbleGrabber, new WobbleGrabber.Grab(false));
		wait(0.4f);

		execute(drivetrain, new Drivetrain.Move(new Vector2(-12f,0f)));

		buffer(wobbleGrabber, new WobbleGrabber.Move(WobbleGrabber.Mode.FOLD));
		execute(drivetrain, new Drivetrain.Move(new Vector2(0f,64f)));

		execute(drivetrain, new Drivetrain.Obstacle(8f));
		execute(drivetrain, new Drivetrain.Drive(Vector2.right, 0.7f));

		wait(0.5f);

		execute(drivetrain, new Drivetrain.Reset());
		execute(drivetrain, new Drivetrain.Drive(Vector2.zero));

		powerShots();
		execute(drivetrain, new Drivetrain.Line());
	}
}