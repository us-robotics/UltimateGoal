package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Behaviors.InertialMeasurementUnit;
import org.firstinspires.ftc.teamcode.Behaviors.Drivetrain;
import org.firstinspires.ftc.teamcode.Behaviors.WobbleGrabber;

import java.security.KeyStore;
import java.util.List;

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
		behaviorList.add(new InertialMeasurementUnit(this));
	}

	@Override
	protected void queueJobs()
	{
		DrivetrainAuto drivetrain = getBehavior(DrivetrainAuto.class);
		WobbleGrabberAuto wobbleGrabber = getBehavior(WobbleGrabberAuto.class);

		grabWobble(10f, 9f);

		execute(drivetrain, (DrivetrainAuto.Job)new DrivetrainAuto.Move(new Vector2(0f, 94f)));
		execute(wobbleGrabber, (WobbleGrabberAuto.Job)new WobbleGrabberAuto.Move(-1));

		execute(drivetrain, (DrivetrainAuto.Job)new DrivetrainAuto.Move(new Vector2(20f, 0f)));
		execute(wobbleGrabber, (WobbleGrabberAuto.Job)new WobbleGrabberAuto.Move(-1, true));

		wait(0.5f);

		execute(drivetrain, (DrivetrainAuto.Job)new DrivetrainAuto.Move(new Vector2(0f, -20f)));
		execute(wobbleGrabber, (WobbleGrabberAuto.Job)new WobbleGrabberAuto.Move(-1));

		execute(drivetrain, (DrivetrainAuto.Job)new DrivetrainAuto.Move(new Vector2(8f, 0)));
		execute(wobbleGrabber, (WobbleGrabberAuto.Job)new WobbleGrabberAuto.Move(0));

		execute(drivetrain, (DrivetrainAuto.Job)new DrivetrainAuto.Move(new Vector2(0f, -64f)));
		execute(drivetrain, (DrivetrainAuto.Job)new DrivetrainAuto.Drive(new Vector2(0f, -1f), 0.5f));

		wait(1f); //Returned back to start
		execute(drivetrain, (DrivetrainAuto.Job)new DrivetrainAuto.Drive(Vector2.zero));
		execute(drivetrain, (DrivetrainAuto.Job)new DrivetrainAuto.Move(new Vector2(0f, 16f), 0.65f));

		grabWobble(2f, 20f);

		execute(drivetrain, (DrivetrainAuto.Job)new DrivetrainAuto.Move(new Vector2(-14f, 0f)));
		execute(drivetrain, (DrivetrainAuto.Job)new DrivetrainAuto.Move(new Vector2(0f, 80)));

		execute(wobbleGrabber, (WobbleGrabberAuto.Job)new WobbleGrabberAuto.Move(-1));
		execute(drivetrain, (DrivetrainAuto.Job)new DrivetrainAuto.Move(new Vector2(6f, 0f)));

		execute(wobbleGrabber, (WobbleGrabberAuto.Job)new WobbleGrabberAuto.Move(-1, true));
		wait(0.5f);

		execute(drivetrain, (DrivetrainAuto.Job)new DrivetrainAuto.Move(new Vector2(0f, -24f)));
	}

	private void grabWobble(float leftDistance, float rightDistance)
	{
		DrivetrainAuto drivetrain = getBehavior(DrivetrainAuto.class);
		WobbleGrabberAuto wobbleGrabber = getBehavior(WobbleGrabberAuto.class);

		execute(wobbleGrabber, (WobbleGrabberAuto.Job)new WobbleGrabberAuto.Move(-1));
		execute(drivetrain, (DrivetrainAuto.Job)new DrivetrainAuto.Move(new Vector2(-leftDistance, 0f), 0.75f));

		execute(wobbleGrabber, (WobbleGrabberAuto.Job)new WobbleGrabberAuto.Reset());
		execute(wobbleGrabber, (WobbleGrabberAuto.Job)new WobbleGrabberAuto.Move(1));

		wait(0.35f);
		execute(wobbleGrabber, (WobbleGrabberAuto.Job)new WobbleGrabberAuto.Move(0, true));

		wait(0.5f);
		execute(drivetrain, (DrivetrainAuto.Job)new DrivetrainAuto.Move(new Vector2(rightDistance, 0f), 0.65f));
		execute(wobbleGrabber, (WobbleGrabberAuto.Job)new WobbleGrabberAuto.Move(0));

		wait(0.3f);
		execute(wobbleGrabber, (WobbleGrabberAuto.Job)new WobbleGrabberAuto.Move(1));
	}
}