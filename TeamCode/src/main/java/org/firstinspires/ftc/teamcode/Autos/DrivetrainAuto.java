package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Behaviors.Drivetrain;

import FTCEngine.Core.Auto.AutoBehavior;
import FTCEngine.Core.OpModeBase;
import FTCEngine.Core.Telemetry;
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
	private float targetAngle;

	@Override
	public void onJobAdded()
	{
		super.onJobAdded();
		Job job = getCurrentJob();

		if (job instanceof Move) drivetrain.resetMotorPositions();
		if (job instanceof Rotate) targetAngle = drivetrain.getAngle() + ((Rotate)job).angle;
	}

	@Override
	protected void updateJob()
	{
		Job job = getCurrentJob();

		if (job instanceof Move)
		{
			Move move = (Move)job;

			final float Cushion = 150f;
			final float Threshold = Cushion * 0.24f;

			float difference = move.distance - drivetrain.getAveragePosition();

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

			final float Cushion = 24f;
			final float Threshold = 8f;

			float difference = Mathf.toSignedAngle(targetAngle - drivetrain.getAngle());

			if (Math.abs(difference) < Threshold)
			{
				drivetrain.setTargetAngle(targetAngle);

				difference = 0f;
				rotate.finishJob();
			}

			int direction = -Mathf.normalize(difference);

			difference = (float)Math.pow(Mathf.clamp01(Math.abs(difference) / Cushion), 1.2f);
			drivetrain.setDirectInputs(Vector2.zero, difference * rotate.power * direction);
		}
	}

	static abstract class Job extends FTCEngine.Core.Auto.Job
	{
	}

	public static class Move extends Job
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

	public static class Drive extends Job //Set drive direction without encoders
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

	public static class Rotate extends Job
	{
		public Rotate(float angle)
		{
			this(angle, 0.65f);
		}

		public Rotate(float angle, float power)
		{
			this.angle = angle;
			this.power = power;
		}

		public final float angle;
		public final float power;
	}
}
