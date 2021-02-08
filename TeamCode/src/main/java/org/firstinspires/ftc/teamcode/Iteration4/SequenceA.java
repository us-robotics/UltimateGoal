package org.firstinspires.ftc.teamcode.Iteration4;

import org.firstinspires.ftc.teamcode.Behaviors.Drivetrain;
import org.firstinspires.ftc.teamcode.Behaviors.Intake;
import org.firstinspires.ftc.teamcode.Behaviors.Launcher;
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
		Launcher launcher = opMode.getBehavior(Launcher.class);
		Intake intake = opMode.getBehavior(Intake.class);

		//execute(wobbleGrabber, new WobbleGrabber.Move(Position.RESET));

		//grabWobble(10f, 9f);

		execute(drivetrain, new Drivetrain.Move(new Vector2(0f, 104f)));


		execute(drivetrain, new Drivetrain.Move(new Vector2(20f, 0f)));
		execute(drivetrain, new Drivetrain.Rotate(-90f));
		execute(drivetrain, new Drivetrain.Rotate(-90f));

		wait(0.4f);

		//Hit wall to Reset Y
		execute(drivetrain, new Drivetrain.Move(new Vector2(0f, -12f)));
		execute(drivetrain, new Drivetrain.Drive(new Vector2(0f, -1f), 0.5f));
		wait(0.7f);

		execute(drivetrain, new Drivetrain.Drive(Vector2.zero));
		execute(drivetrain, new Drivetrain.Reset());

		execute(launcher, new Launcher.Prime(Launcher.HIGH_POWER - 0.02f, true));
		execute(drivetrain, new Drivetrain.Move(new Vector2(-10f, 0f)));

		//Forward to launch position
		execute(drivetrain, new Drivetrain.Move(new Vector2(0f, 53f)));
		execute(wobbleGrabber, new WobbleGrabber.Move(WobbleGrabber.Position.GRAB));
		execute(wobbleGrabber, new WobbleGrabber.Grab(false));
		execute(wobbleGrabber, new WobbleGrabber.Move(WobbleGrabber.Position.RESET));
		execute(drivetrain, new Drivetrain.Move(new Vector2(0f,6f)));

		wait(0.5f);

		execute(drivetrain, new Drivetrain.Move(new Vector2(25f, 0f)));
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
	}
}
