package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Behaviors.Drivetrain;

import FTCEngine.Core.Auto.AutoBehavior;
import FTCEngine.Core.OpModeBase;
import FTCEngine.Math.Mathf;
import FTCEngine.Math.Vector2;

public class DrivetrainAuto extends AutoBehavior<DrivetrainAuto.Job>
{
	public DrivetrainAuto(OpModeBase opMode)
	{
		super(opMode);
	}

	@Override
	public void awake(HardwareMap hardwareMap)
	{
		super.awake(hardwareMap);
		drivetrain = opMode.getBehavior(Drivetrain.class);
	}

	//following Gary's syntax and declaring down here to separate methods
	private Drivetrain drivetrain;

	@Override
	public void onJobAdded()
	{
		super.onJobAdded();
		Job job = getCurrentJob();

		if (job instanceof Move) drivetrain.resetMotorPositions();
	}

	@Override
	protected void updateJob()
	{
		Job job = getCurrentJob();

		if (job instanceof Move)
		{
			Move move = (Move)job;

			final float Cushion = 100;
			final float Threshold = 0.08f;

			float difference = move.distance - drivetrain.averagePosition();

			if (Math.abs(difference) < Threshold)
			{
				difference = 0f;
				move.finishJob();
			}

			difference = Mathf.clamp(difference / Cushion, -1f, 1f);
			drivetrain.setDirectInputs(move.direction.mul(difference), 0f);
		}

		if (job instanceof Rotate)
		{
			Rotate rotate = (Rotate)job;
		}
	}

	static abstract class Job extends FTCEngine.Core.Auto.Job
	{
	}

	static class Move extends Job
	{
		public Move(Vector2 movement)
		{
			//Can only move perpendicular to the axes
			if (Math.abs(movement.x) > Math.abs(movement.y))
			{
				direction = new Vector2(Mathf.normalize(movement.x), 0f);
				distance = Math.abs(Math.round(movement.x * InchToTickStrafe));
			}
			else
			{
				direction = new Vector2(0f, Mathf.normalize(movement.y));
				distance = Math.abs(Math.round(movement.y * InchToTickForward));
			}
		}

		public final Vector2 direction;
		public final float distance; //Distance in inches

		final float InchToTickForward = 28.169f;
		final float InchToTickStrafe = 0f;
	}

	static class Rotate extends Job
	{

	}
}
