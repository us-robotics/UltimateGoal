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
	private boolean dragging = true;

	@Override
	protected void updateJob()
	{
		Job job = getCurrentJob();

		if (job instanceof Drag)
		{
			Drag drag = (Drag)job;
			dragging = drag.drag;

			apply();
			drag.finishJob();
		}
	}

	private void apply()
	{
		jeff.setPosition(dragging ? 0f : 0.7f);
	}

	abstract static class Job extends FTCEngine.Core.Auto.Job
	{
	}

	public static class Drag extends Job
	{
		public Drag(boolean drag)
		{
			this.drag = drag;
		}

		public final boolean drag;
	}
}
