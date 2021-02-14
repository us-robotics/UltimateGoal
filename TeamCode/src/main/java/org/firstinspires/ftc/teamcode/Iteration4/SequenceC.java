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

		buffer(drivetrain, new Drivetrain.Move(new Vector2(0f, 104f)));
		execute(wobbleGrabber, new WobbleGrabber.Move(WobbleGrabber.Mode.GRAB));

		execute(drivetrain, new Drivetrain.Rotate(90f));
		execute(drivetrain, new Drivetrain.Rotate(90f));

		//Hit wall reset Y
		execute(drivetrain, new Drivetrain.Move(new Vector2(0f,-12)));
		execute(drivetrain, new Drivetrain.Drive(new Vector2(0f, -1f), 0.55f));

		wait(0.75f);
		execute(drivetrain, new Drivetrain.Drive(Vector2.zero));
		execute(drivetrain, new Drivetrain.Reset());

		execute(wobbleGrabber, new WobbleGrabber.Grab(false));
		execute(wobbleGrabber, new WobbleGrabber.Move(WobbleGrabber.Mode.FOLD));

		execute(drivetrain, new Drivetrain.Move(new Vector2(0f,60f)));

		powerShots();
		grabWobble2();

		execute(drivetrain, new Drivetrain.Move(new Vector2(0f,-60f)));
		execute(drivetrain, new Drivetrain.Drive(new Vector2(0f, -1f), 0.55f));

		wait(0.75f);
		execute(drivetrain, new Drivetrain.Drive(Vector2.zero));
		execute(drivetrain, new Drivetrain.Reset());

		execute(drivetrain, new Drivetrain.Move(new Vector2(48f,0f)));

		if (true) return;

		//Grab and place wobble
		grabWobble(4f, 9f);

		execute(drivetrain, new Drivetrain.Move(new Vector2(-6f, 0f)));
		execute(drivetrain, new Drivetrain.Move(new Vector2(0f, 100f)));
		execute(drivetrain, new Drivetrain.Move(new Vector2(10f,0f)));
		execute(drivetrain, new Drivetrain.Rotate(90f));
		execute(drivetrain, new Drivetrain.Rotate(45f));

		wait(0.4f);
		execute(wobbleGrabber, new WobbleGrabber.Move(WobbleGrabber.Mode.GRAB));
		execute(wobbleGrabber, new WobbleGrabber.Grab(false));
		//execute(wobbleGrabber, new WobbleGrabber.Move(Mode));

		execute(drivetrain, new Drivetrain.Rotate(57f));
		execute(launcher, new Launcher.Prime(Launcher.HIGH_POWER - 0.02f, true));

		//Forward to launch position
		execute(drivetrain, new Drivetrain.Move(new Vector2(0f, 59f)));
		wait(0.5f);

		execute(drivetrain, new Drivetrain.Move(new Vector2(17f, 0f)));
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

		wait(0.5f);
		execute(drivetrain, new Drivetrain.Rotate(45f));
		execute(drivetrain, new Drivetrain.Move(new Vector2(0f, 30f)));
	}
}