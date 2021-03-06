package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.robotcore.hardware.HardwareMap;

import FTCEngine.Core.Auto.AutoBehavior;
import FTCEngine.Core.OpModeBase;

public class WobbleGrabberOldAuto extends AutoBehavior<WobbleGrabberOldAuto.Job>
{
	public WobbleGrabberOldAuto(OpModeBase opMode)
	{
		super(opMode);
	}

	@Override
	public void awake(HardwareMap hardwareMap)
	{
		super.awake(hardwareMap);
		grabber = opMode.getBehavior(WobbleGrabberOld.class);
	}

	WobbleGrabberOld grabber;

	@Override
	protected void updateJob()
	{
		Job job = getCurrentJob();
		if (job == null) return;

		if (job instanceof Reset)
		{
			grabber.resetEncoder();
		}
		else if (job instanceof Move)
		{
			Move move = (Move)job;

			if (move.pushing == 0)
			{
				grabber.clearPush();
			}
			else if (move.pushing > 0) grabber.setPushUp();
			else if (move.pushing < 0) grabber.setPushDown();

			grabber.setReleased(move.released);
		}

		job.finishJob();
	}

	abstract static class Job extends FTCEngine.Core.Auto.Job
	{
	}

	public static class Reset extends Job
	{
	}

	public static class Move extends Job
	{
		public Move(int pushing, boolean released)
		{
			this.pushing = pushing;
			this.released = released;
		}


		public Move(int pushing)
		{
			this(pushing, false);
		}

		public final int pushing;
		public final boolean released;
	}
}
