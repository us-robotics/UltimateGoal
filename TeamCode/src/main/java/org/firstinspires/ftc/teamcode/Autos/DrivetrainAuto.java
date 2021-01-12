package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Behaviors.Drivetrain;

import FTCEngine.Core.Auto.AutoBehavior;
import FTCEngine.Core.OpModeBase;
import FTCEngine.Core.Time;
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
	private float rotateStartTime;

	private float getTime()
	{
		return opMode.getHelper(Time.class).getTime();
	}

	@Override
	public void onJobAdded()
	{
		super.onJobAdded();
		Job job = getCurrentJob();

		if (job instanceof Move) drivetrain.resetMotorPositions();
		else if (job instanceof Rotate) rotateStartTime = getTime();
	}

	@Override
	protected void updateJob()
	{
		Job job = getCurrentJob();

		if (job instanceof Move)
		{
			Move move = (Move)job;

			final float Cushion = 150;
			final float Threshold = Cushion * 0.24f;

			float difference = move.distance - drivetrain.averagePosition();

			if (Math.abs(difference) < Threshold)
			{
				difference = 0f;
				move.finishJob();
			}

			difference = Mathf.clamp(difference / Cushion, -move.maxPower, move.maxPower);
			drivetrain.setDirectInputs(move.direction.mul(difference), 0f);
		}

		if (job instanceof Drive)
		{
			Drive drive = (Drive)job;

			drivetrain.setDirectInputs(drive.direction.mul(drive.maxPower), 0f);
			drive.finishJob();
		}

		if (job instanceof Rotate)
		{
			Rotate rotate = (Rotate)job;

			if (getTime() - rotateStartTime >= 1f)
			{
				//Rotation end
				drivetrain.setDirectInputs(Vector2.zero, 0f);
				rotate.finishJob();
			}
			else drivetrain.setDirectInputs(Vector2.zero, 0.8f);
		}
	}

	static abstract class Job extends FTCEngine.Core.Auto.Job
	{
	}

	static class Move extends Job
	{
		/**
		 * Creates a move job. Only the most significant axis will be used (Can only move perpendicular to the axes)
		 *
		 * @param movement Movement in inches; the less significant component will be discarded.
		 */
		public Move(Vector2 movement)
		{
			this(movement, 1f);
		}

		/**
		 * Creates a move job. Only the most significant axis will be used (Can only move perpendicular to the axes)
		 *
		 * @param movement Movement in inches; the less significant component will be discarded.
		 */
		public Move(Vector2 movement, float maxPower)
		{
			if (Math.abs(movement.x) > Math.abs(movement.y))
			{
				direction = new Vector2(Mathf.normalize(movement.x), 0f);
				distance = Math.abs(movement.x * InchToTickStrafe);
			}
			else
			{
				direction = new Vector2(0f, Mathf.normalize(movement.y));
				distance = Math.abs(movement.y * InchToTickForward);
			}

			this.maxPower = Mathf.clamp01(maxPower);
		}

		public final Vector2 direction;
		public final float distance; //Distance in encoder ticks
		public final float maxPower;

		final float InchToTickForward = 26.6567937801f;
		final float InchToTickStrafe = 32.8767123288f;
	}

	static class Drive extends Job //Set drive direction without encoders
	{
		public Drive(Vector2 direction)
		{
			this(direction, 1f);
		}

		public Drive(Vector2 direction, float maxPower)
		{
			this.direction = direction;
			this.maxPower = maxPower;
		}

		public final Vector2 direction;
		public final float maxPower;
	}

	static class Rotate extends Job
	{

	}
}
