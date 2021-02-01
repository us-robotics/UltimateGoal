package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
	private float targetAngle;
	private boolean rotated;

	private Vector2 positionalInput = Vector2.zero;
	private float rotationalInput = 0f;

	@Override
	public void start()
	{
		super.start();
		targetAngle = getAngle();
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

			if (opMode.input.getTrigger(Input.Source.CONTROLLER_1, Input.Button.LEFT_TRIGGER) > 0.1f)
			{
				exponent = 0.64f;
				multiplier = 0.48f;
			}

			positionalInput = positionalInput.normalize().mul((float)Math.pow(positionalInput.getMagnitude(), exponent) * multiplier);
			rotationalInput = Mathf.normalize(rotationalInput) * (float)Math.pow(Math.abs(rotationalInput), exponent) * multiplier;
		}

		//If no rotational input, then IMU is used to counterbalance hardware inaccuracy to drive straight
		if (!positionalInput.equals(Vector2.zero)) rotated = false;
		if (!Mathf.almostEquals(rotationalInput, 0f)) rotated = true;

		if (!opMode.hasSequence())
		{
			if (opMode.input.getTrigger(Input.Source.CONTROLLER_1, Input.Button.RIGHT_TRIGGER) > 0.1f) rotated = true;
		}

		if (rotated)
		{
			targetAngle = getAngle();
			setRawVelocities(positionalInput, rotationalInput);
		}
		else
		{
			float deviation = Mathf.toSignedAngle(getAngle() - targetAngle);
			setRawVelocities(positionalInput, deviation / 25f);
		}
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

//		opMode.getHelper(Telemetry.class).addData("Powers", Arrays.toString(powers));
	}

	@Override
	public void onJobAdded()
	{
		super.onJobAdded();
		Drivetrain.Job job = getCurrentJob();

		if (job instanceof Drivetrain.Move) resetMotorPositions();
		if (job instanceof Drivetrain.Rotate) targetAngle = getAngle() + ((Drivetrain.Rotate)job).angle;
	}

	@Override
	protected void updateJob()
	{
		Drivetrain.Job job = getCurrentJob();

		if (job instanceof Drivetrain.Move)
		{
			Drivetrain.Move move = (Drivetrain.Move)job;

			final float Cushion = 150f;
			final float Threshold = Cushion * 0.24f;

			float difference = move.distance - getAveragePosition();

			if (Math.abs(difference) < Threshold)
			{
				difference = 0f;
				move.finishJob();
			}

			difference = Mathf.clamp(difference / Cushion, -move.maxPower, move.maxPower);
			setDirectInputs(move.direction.mul(difference), 0f);
		}

		if (job instanceof Drivetrain.Drive)
		{
			Drivetrain.Drive drive = (Drivetrain.Drive)job;

			setDirectInputs(drive.direction.mul(drive.maxPower), 0f);
			drive.finishJob();
		}

		if (job instanceof Drivetrain.Rotate)
		{
			Drivetrain.Rotate rotate = (Drivetrain.Rotate)job;

			final float Cushion = 22f;
			final float Threshold = 5f;

			float difference = Mathf.toSignedAngle(targetAngle - getAngle());

			if (Math.abs(difference) < Threshold)
			{
				difference = 0f;
				rotate.finishJob();
			}

			int direction = -Mathf.normalize(difference);

			difference = (float)Math.pow(Mathf.clamp01(Math.abs(difference) / Cushion), 1.6f);
			setDirectInputs(Vector2.zero, difference * rotate.power * direction);
		}

		if (job instanceof Drivetrain.Reset)
		{
			Drivetrain.Reset reset = (Drivetrain.Reset)job;

			targetAngle = getAngle();
			reset.finishJob();
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
			sum += motor.getCurrentPosition();
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
			this(angle, 0.6f);
		}

		public Rotate(float angle, float power)
		{
			this.angle = angle;
			this.power = power;
		}

		public final float angle;
		public final float power;
	}

	/**
	 * Resets internal IMU angle
	 */
	public static class Reset extends Drivetrain.Job
	{

	}
}
