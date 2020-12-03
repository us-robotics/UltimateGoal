package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Behaviors.InertialMeasurementUnit;
import org.firstinspires.ftc.teamcode.Behaviors.Drivetrain;
import org.firstinspires.ftc.teamcode.Behaviors.WobbleGrabber;

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

		execute(wobbleGrabber, new WobbleGrabberAuto.Job(1, false));
		execute(drivetrain, (DrivetrainAuto.Job)new DrivetrainAuto.Move(new Vector2(78f, 0f)));

		execute(wobbleGrabber, new WobbleGrabberAuto.Job(-1, false));
		wait(1f);

		execute(wobbleGrabber, new WobbleGrabberAuto.Job(-1, true));
		execute(drivetrain, (DrivetrainAuto.Job)new DrivetrainAuto.Move(new Vector2(-78f, 0f)));

		//Back to start
		execute(drivetrain, (DrivetrainAuto.Job)new DrivetrainAuto.Move(new Vector2(0f, -15f)));
		execute(wobbleGrabber, new WobbleGrabberAuto.Job(1, true));

		wait(0.1f);
		execute(drivetrain, (DrivetrainAuto.Job)new DrivetrainAuto.Move(new Vector2(10f, 0f)));
		execute(wobbleGrabber, new WobbleGrabberAuto.Job(1, false));

		execute(drivetrain, (DrivetrainAuto.Job)new DrivetrainAuto.Move(new Vector2(64f, 0f)));
	}
}