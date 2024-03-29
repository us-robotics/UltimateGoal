package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import FTCEngine.Core.Auto.AutoBehavior;
import FTCEngine.Core.Input;
import FTCEngine.Core.OpModeBase;
import FTCEngine.Math.Mathf;
import FTCEngine.Math.Vector2;

public class Drivetrain extends AutoBehavior<Drivetrain.Job>
{
	/**
	 * NOTE: Do not configure the electronics in the constructor, do them in the awake method!
	 */
	public Drivetrain(OpModeBase opMode)
	{
		super(opMode);
	}

	@Override
	public void awake(HardwareMap hardwareMap)
	{
		super.awake(hardwareMap);

		motors[0] = hardwareMap.dcMotor.get("frontRight");
		motors[1] = hardwareMap.dcMotor.get("frontLeft");
		motors[2] = hardwareMap.dcMotor.get("backRight");
		motors[3] = hardwareMap.dcMotor.get("backLeft");

		motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
		motors[2].setDirection(DcMotorSimple.Direction.REVERSE);

		for (DcMotor motor : motors) motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		imu = opMode.getBehavior(InertialMeasurementUnit.class);
		if (imu == null) throw new IllegalStateException("No imu behavior found!");

		resetMotorPositions();
		setRawVelocities(Vector2.zero, 0f);
	}

	private final DcMotor[] motors = new DcMotor[4];
	private final float[] powers = new float[4];

	private InertialMeasurementUnit imu;
	private boolean angleCorrection;
	private float persistentAngle;

	private Vector2 positionalInput = Vector2.zero;
	private float rotationalInput = 0f;

	@Override
	public void start()
	{
		super.start();
		persistentAngle = getAngle();
	}

	@Override
	public void update()
	{
		super.update();

		if (!opMode.hasSequence())
		{
			//Process input if is not in auto
			positionalInput = opMode.input.getVector(Input.Source.CONTROLLER_1, Input.Button.LEFT_JOYSTICK);
			rotationalInput = opMode.input.getVector(Input.Source.CONTROLLER_1, Input.Button.RIGHT_JOYSTICK).x;

			//Process input for smoother control by interpolating a polynomial curve
			float exponent = 0.72f;
			float multiplier = 1f;

			if (opMode.input.getTrigger(Input.Source.CONTROLLER_1, Input.Button.LEFT_TRIGGER) > 0.4f)
			{
				exponent = 0.62f;
				multiplier = 0.44f;
			}

			positionalInput = positionalInput.normalize().mul((float)Math.pow(positionalInput.getMagnitude(), exponent) * multiplier);
			rotationalInput = Mathf.normalize(rotationalInput) * (float)Math.pow(Math.abs(rotationalInput), exponent) * multiplier;
		}

		//If no rotational input, then IMU is used to counterbalance hardware inaccuracy to drive straight
		if (!positionalInput.equals(Vector2.zero)) angleCorrection = true;
		if (!Mathf.almostEquals(rotationalInput, 0f)) angleCorrection = false;

		if (!opMode.hasSequence())
		{
			if (opMode.input.getTrigger(Input.Source.CONTROLLER_1, Input.Button.RIGHT_TRIGGER) > 0.4f) angleCorrection = false;
		}

		if (angleCorrection)
		{
			float deviation = Mathf.toSignedAngle(getAngle() - persistentAngle);
			setRawVelocities(positionalInput, Mathf.clamp(deviation / 32f, -0.7f, 0.7f));
		}
		else
		{
			persistentAngle = getAngle();
			setRawVelocities(positionalInput, rotationalInput);
		}

//		opMode.debug.addData("Position", getAveragePosition());
	}

	private void setRawVelocities(Vector2 localDirection, float angularDelta)
	{
		localDirection = localDirection.clampMagnitude(0f, 1f);
		angularDelta = Mathf.clamp(angularDelta, -1f, 1f);

		powers[0] = localDirection.y - localDirection.x - angularDelta;
		powers[1] = localDirection.y + localDirection.x + angularDelta;
		powers[2] = localDirection.y + localDirection.x - angularDelta;
		powers[3] = localDirection.y - localDirection.x + angularDelta;

		float max = -Float.MAX_VALUE;

		for (int i = 0; i < powers.length; i++)
		{
			float power = Math.abs(powers[i]);

			if (power < 0.05f) powers[i] = 0f;
			else max = Math.max(max, power);
		}

		max = Math.max(max, 1f); //Scales all powers down by a multiplier if one power is higher than 1
		for (int i = 0; i < motors.length; i++) motors[i].setPower(powers[i] / max);

//		opMode.debug.addData("Powers", Arrays.toString(powers));
	}

	@Override
	public void onJobAdded()
	{
		super.onJobAdded();
		Job job = getCurrentJob();

		if (job instanceof Move) resetMotorPositions();

		if (job instanceof Rotate)
		{
			Rotate rotate = (Rotate)getCurrentJob();
			rotate.targetAngle = persistentAngle + rotate.angle;
		}
	}

	@Override
	protected void updateJob()
	{
		Job job = getCurrentJob();

		if (job instanceof Move)
		{
			Move move = (Move)job;

			final float Cushion = 170f;
			final float Threshold = Cushion * 0.22f;

			float difference = move.distance - getAveragePosition();

			if (difference < Threshold)
			{
				difference = 0f;
				move.finishJob();
			}

			float power = Math.min(difference / Cushion, move.maxPower);
			setDirectInputs(move.direction.normalize().mul(power), 0f);
		}

		if (job instanceof Drive)
		{
			Drive drive = (Drive)job;

			setDirectInputs(drive.direction.mul(drive.maxPower), 0f);
			drive.finishJob();
		}

		if (job instanceof Rotate)
		{
			Rotate rotate = (Rotate)job;

			final float Threshold = 8f;
			final float Cushion = 42f;

			final float MinPower = 0.21f;
			final float MaxPower = 0.47f;

			float difference = Mathf.toSignedAngle(rotate.targetAngle - getAngle());

			if (Math.abs(difference) < Threshold)
			{
				float current = opMode.time.getTime();
				final float Time = 0.12f;

				if (Mathf.almostEquals(rotate.startTime, 0f)) rotate.startTime = current;
				else if (rotate.startTime + Time < current)
				{
					persistentAngle = rotate.targetAngle;
					rotate.finishJob();
				}

				setDirectInputs(Vector2.zero, 0f);
			}
			else
			{
				rotate.startTime = 0f;

				float power = Mathf.lerp(MinPower, MaxPower, Mathf.sigmoid(Math.abs(difference) / Cushion));
				setDirectInputs(Vector2.zero, power * -Mathf.normalize(difference));
			}
		}

		if (job instanceof Reset)
		{
			Reset reset = (Reset)job;

			persistentAngle = getAngle();
			reset.finishJob();
		}

		if (job instanceof Obstacle)
		{
			Obstacle obstacle = (Obstacle)job;
			DistanceSensors sensor = opMode.getBehavior(DistanceSensors.class);

			final float Threshold = 1.8f;
			final float Cushion = 12f;

			final float MinPower = 0.22f;
			final float MaxPower = 0.46f;

			float difference = sensor.getDistance() - obstacle.distance;

			if (Math.abs(difference) < Threshold)
			{
				float current = opMode.time.getTime();
				final float Time = 0.12f;

				if (Mathf.almostEquals(obstacle.startTime, 0f)) obstacle.startTime = current;
				else if (obstacle.startTime + Time < current) obstacle.finishJob();

				setDirectInputs(Vector2.zero, 0f);
			}
			else
			{
				obstacle.startTime = 0f;

				float power = Mathf.lerp(MinPower, MaxPower, Mathf.sigmoid(Math.abs(difference) / Cushion));
				setDirectInputs(new Vector2(power * Mathf.normalize(difference), 0f), 0f);
			}
		}

		if (job instanceof Line)
		{
			Line line = (Line)job;
			ColorSensors color = opMode.getBehavior(ColorSensors.class);

			boolean front = color.getLineFront() == line.color;
			boolean back = color.getLineBack() == ColorSensors.Line.NONE;

			int y = front ? 1 : back ? -1 : 0;

			if (y == 0)
			{
//				float current = opMode.time.getTime();
//				final float Time = 0.07f;
//
//				if (Mathf.almostEquals(line.startTime, 0f)) line.startTime = current;
//				else if (line.startTime + Time < current) line.finishJob();
//

				line.finishJob();
				setDirectInputs(Vector2.zero, 0f);
			}
			else
			{
				line.startTime = 0f;
				final float CorrectPower = 0.2f;

				setDirectInputs(new Vector2(0f, y * CorrectPower), 0f);
			}
		}
	}

	private void setDirectInputs(Vector2 positionalInput, float rotationalInput)
	{
		this.positionalInput = positionalInput;
		this.rotationalInput = rotationalInput;
	}

	private void resetMotorPositions()
	{
		for (DcMotor motor : motors)
		{
			motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		}
	}

	private float getAveragePosition()
	{
		float sum = 0f;

		for (DcMotor motor : motors)
		{
			sum += Math.abs(motor.getCurrentPosition());
		}

		return sum / motors.length;
	}

	private float getAngle()
	{
		return Mathf.toSignedAngle(imu.getAngles().z);
	}

	static abstract class Job extends FTCEngine.Core.Auto.Job
	{
	}

	public static class Move extends Drivetrain.Job
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

	public static class Drive extends Drivetrain.Job //Set drive direction without encoders
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

	public static class Rotate extends Drivetrain.Job
	{
		public Rotate(float angle)
		{
			this.angle = angle;
		}

		public final float angle;

		private float targetAngle;
		private float startTime;
	}

	/**
	 * Resets internal IMU angle
	 */
	public static class Reset extends Drivetrain.Job
	{

	}

	/**
	 * Strafes the drivetrain so its side is a certain distance to an obstacle.
	 */
	public static class Obstacle extends Drivetrain.Job
	{
		public Obstacle(float distance)
		{
			this.distance = distance;
		}

		public final float distance;
		private float startTime;
	}

	/**
	 * Drives the drivetrain backward until it hits a line
	 */
	public static class Line extends Drivetrain.Job
	{
		public Line()
		{
			this(ColorSensors.Line.WHITE);
		}

		public Line(ColorSensors.Line color)
		{
			this.color = color;
		}

		public final ColorSensors.Line color;
		private float startTime;
	}
}
