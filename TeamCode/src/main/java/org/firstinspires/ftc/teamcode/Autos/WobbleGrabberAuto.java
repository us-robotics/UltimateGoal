package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Behaviors.WobbleGrabber;

import FTCEngine.Core.Auto.AutoBehavior;
import FTCEngine.Core.OpModeBase;

public class WobbleGrabberAuto extends AutoBehavior<WobbleGrabberAuto.Job>
{
	public WobbleGrabberAuto(OpModeBase opMode)
	{
		super(opMode);
	}

	@Override
	public void awake(HardwareMap hardwareMap)
	{
		super.awake(hardwareMap);
		grabber = opMode.getBehavior(WobbleGrabber.class);
	}

	WobbleGrabber grabber;

	@Override
	protected void updateJob()
	{
		Job job = getCurrentJob();
		if (job == null) return;

		switch (job.pushing)
		{
			case -1:
			{
				grabber.setPushDown();
				break;
			}
			case 0:
			{
				grabber.clearPush();
				break;
			}
			case 1:
			{
				grabber.setPushUp();
				break;
			}
		}

		grabber.setReleased(job.released);
		job.finishJob();
	}

	public static class Job extends FTCEngine.Core.Auto.Job
	{
		public Job(int pushing, boolean released)
		{
			this.pushing = pushing;
			this.released = released;
		}
		public final int pushing;
		public final boolean released;
	}
}
