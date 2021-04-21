package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import FTCEngine.Core.Auto.AutoBehavior;
import FTCEngine.Core.OpModeBase;

public class WobbleDraggerI5 extends AutoBehavior<WobbleDraggerI5.Job>
{
	public WobbleDraggerI5(OpModeBase opMode)
	{
		super(opMode);
	}

	@Override
	public void awake(HardwareMap hardwareMap)
	{
		super.awake(hardwareMap);
		jeff = hardwareMap.servo.get("jeff");

		apply();
	}

	private Servo jeff;
	private boolean grabbed = true;

	@Override
	protected void updateJob()
	{
		Job job = getCurrentJob();

		if (job instanceof Grab)
		{
			Grab grab = (Grab)job;
			grabbed = grab.grab;

			apply();
			grab.finishJob();
		}
	}

	private void apply()
	{
		jeff.setPosition(grabbed ? 0f : 0.7f);
	}

	abstract static class Job extends FTCEngine.Core.Auto.Job
	{
	}

	public static class Grab extends Job
	{
		public Grab(boolean grab)
		{
			this.grab = grab;
		}

		public final boolean grab;
	}
}
