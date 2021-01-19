package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Behaviors.InertialMeasurementUnit;
import org.firstinspires.ftc.teamcode.Behaviors.Drivetrain;
import org.firstinspires.ftc.teamcode.Behaviors.Launcher;
import org.firstinspires.ftc.teamcode.Behaviors.WobbleGrabber;

import java.security.KeyStore;
import java.util.List;

import FTCEngine.Core.Auto.Alliance;
import FTCEngine.Core.Auto.AutoOpModeBase;
import FTCEngine.Core.Behavior;
import FTCEngine.Math.Vector2;

@Autonomous(name = "MecanumI4 Auto")
public class MecanumI4Auto extends AutoOpModeBase
{
	@Override
	public void addBehaviors(List<Behavior> behaviorList)
	{
		behaviorList.add(new Drivetrain(this));
		behaviorList.add(new DrivetrainAuto(this));
		behaviorList.add(new WobbleGrabber(this));
		behaviorList.add(new WobbleGrabberAuto(this));
		behaviorList.add(new Launcher(this));
		behaviorList.add(new LauncherAuto(this));
		behaviorList.add(new InertialMeasurementUnit(this));
	}

	@Override
	protected void queueJobs()
	{
		DrivetrainAuto drivetrain = getBehavior(DrivetrainAuto.class);
		WobbleGrabberAuto wobbleGrabber = getBehavior(WobbleGrabberAuto.class);
		LauncherAuto launcher = getBehavior(LauncherAuto.class);

		grabWobble(10f, 9f);

		execute(drivetrain, new DrivetrainAuto.Move(new Vector2(0f, 80f)));
		execute(wobbleGrabber, new WobbleGrabberAuto.Move(0));

		execute(drivetrain, new DrivetrainAuto.Move(new Vector2(20f, 0f)));
		execute(drivetrain, new DrivetrainAuto.Rotate(180f));

		execute(wobbleGrabber, new WobbleGrabberAuto.Move(1));
		wait(0.4f);

		//Hit wall to Reset Y
		execute(drivetrain, new DrivetrainAuto.Move(new Vector2(0f, -36f)));
		execute(drivetrain, new DrivetrainAuto.Drive(new Vector2(0f, -1f), 0.5f));
		wait(1f);

		execute(drivetrain, new DrivetrainAuto.Drive(Vector2.zero));
		execute(wobbleGrabber, new WobbleGrabberAuto.Move(-1, true));

		execute(drivetrain, new DrivetrainAuto.Move(new Vector2(-10f, 0f)));
		execute(wobbleGrabber, new WobbleGrabberAuto.Move(-1));

		//Forward to launch position
		execute(launcher, new LauncherAuto.Prime(1f));

		execute(drivetrain, new DrivetrainAuto.Move(new Vector2(0f, 64f)));
		execute(drivetrain, new DrivetrainAuto.Move(new Vector2(24f, 0f)));

		execute(drivetrain, new DrivetrainAuto.Drive(new Vector2(1f, 0f), 0.5f));

		wait(0.7f);
		execute(drivetrain, new DrivetrainAuto.Drive(Vector2.zero));

		for (int i = 0; i < 3; i++)
		{
			execute(launcher, new LauncherAuto.Hit(true));
			wait(1f);

			execute(launcher, new LauncherAuto.Hit(false));
			wait(1f);
		}

		execute(launcher, new LauncherAuto.Prime(0f));

		//Move to grab second wobble
		execute(drivetrain, new DrivetrainAuto.Move(new Vector2(-64f, 0f)));
		execute(drivetrain, new DrivetrainAuto.Move(new Vector2(0f, 60f)));

		grabWobble(5f, 10f);

		execute(drivetrain, new DrivetrainAuto.Move(new Vector2(0f, -72f)));
		execute(drivetrain, new DrivetrainAuto.Move(new Vector2(0f, 36f)));

		//Drop second wobble
		execute(wobbleGrabber, new WobbleGrabberAuto.Move(-1));
		wait(0.5f);

		execute(wobbleGrabber, new WobbleGrabberAuto.Move(0, true));
		wait(0.5f);

		execute(drivetrain, new DrivetrainAuto.Move(new Vector2(-16f, 0f)));
		execute(wobbleGrabber, new WobbleGrabberAuto.Move(-1));
	}

	private void grabWobble(float leftDistance, float rightDistance)
	{
		DrivetrainAuto drivetrain = getBehavior(DrivetrainAuto.class);
		WobbleGrabberAuto wobbleGrabber = getBehavior(WobbleGrabberAuto.class);

		execute(wobbleGrabber, new WobbleGrabberAuto.Move(-1));
		execute(drivetrain, new DrivetrainAuto.Move(new Vector2(-leftDistance, 0f), 0.75f));

		execute(wobbleGrabber, new WobbleGrabberAuto.Reset());
		execute(wobbleGrabber, new WobbleGrabberAuto.Move(1));

		wait(0.35f);
		execute(wobbleGrabber, new WobbleGrabberAuto.Move(0, true));

		wait(0.5f);
		execute(drivetrain, new DrivetrainAuto.Move(new Vector2(rightDistance, 0f), 0.65f));
		execute(wobbleGrabber, new WobbleGrabberAuto.Move(0));

		wait(0.3f);
		execute(drivetrain, new DrivetrainAuto.Move(new Vector2(-4, 0f), 0.75f));
		execute(wobbleGrabber, new WobbleGrabberAuto.Move(1));
	}
}