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

		//drive to square, let go, reset on wall ed
		execute(drivetrain, new Drivetrain.Move(new Vector2(0f, 100f)));
		execute(wobbleGrabber, new WobbleGrabber.Move(WobbleGrabber.Position.GRAB));
		execute(wobbleGrabber, new WobbleGrabber.Grab(false));
		wait(.2f);
		execute(wobbleGrabber, new WobbleGrabber.Move(WobbleGrabber.Position.FOLD));
		execute(drivetrain, new Drivetrain.Drive(new Vector2(-1f, 0f), .7f));
		wait(.8f);
		execute(drivetrain, new Drivetrain.Drive(Vector2.zero));
		execute(drivetrain, new Drivetrain.Reset());

		execute(drivetrain, new Drivetrain.Move(new Vector2(8f, 0)));
		execute(drivetrain, new Drivetrain.Rotate(180f));
		execute(drivetrain, new Drivetrain.Move(new Vector2(0f, 28f)));
		execute(drivetrain, new Drivetrain.Drive(new Vector2(1f, 0f), .7f));
		wait(.8f);
		execute(drivetrain, new Drivetrain.Drive(Vector2.zero));
		execute(drivetrain, new Drivetrain.Reset());

		//Launch ring 1
		execute(launcher, new Launcher.Prime(Launcher.HIGH_POWER, true));
		wait(1f);
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

		execute(drivetrain, new Drivetrain.Move(new Vector2(-24f, 0f)));
		execute(drivetrain, new Drivetrain.Rotate(180f));
		execute(drivetrain, new Drivetrain.Move(new Vector2(0f, -80f)));
		execute(drivetrain, new Drivetrain.Drive(new Vector2(0f, -1f), .7f));
		wait(.8f);
		execute(drivetrain, new Drivetrain.Drive(Vector2.zero));
		execute(drivetrain, new Drivetrain.Reset());

		grabWobble(20f, 9f);

		execute(drivetrain, new Drivetrain.Move(new Vector2(0f, -108f)));


	}
}