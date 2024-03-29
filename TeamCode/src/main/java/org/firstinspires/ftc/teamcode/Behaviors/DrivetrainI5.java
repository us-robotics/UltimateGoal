package org.firstinspires.ftc.teamcode.Behaviors;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import FTCEngine.Core.Auto.AutoBehavior;
import FTCEngine.Core.Input;
import FTCEngine.Core.OpModeBase;
import FTCEngine.Math.Mathf;
import FTCEngine.Math.Vector2;

public class DrivetrainI5 extends AutoBehavior<DrivetrainI5.Job>
{
	/**
	 * NOTE: Do not configure the electronics in the constructor, do them in the awake method!
	 */
	public DrivetrainI5(OpModeBase opMode)
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
		drive = new SampleMecanumDrive(hardwareMap);

		if (imu == null) throw new IllegalStateException("No imu behavior found!");

		resetMotorPositions();
		setRawVelocities(Vector2.zero, 0f);
	}

	private final DcMotor[] motors = new DcMotor[4];
	private final float[] powers = new float[4];

	private InertialMeasurementUnit imu;
	private boolean angleCorrection;
	private float persistentAngle;

	private SampleMecanumDrive drive;

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

		if (opMode.hasSequence()) return;

		//Process input if is not in auto
		Vector2 positionalInput = opMode.input.getVector(Input.Source.CONTROLLER_1, Input.Button.LEFT_JOYSTICK);
		float rotationalInput = opMode.input.getVector(Input.Source.CONTROLLER_1, Input.Button.RIGHT_JOYSTICK).x;

		//Process input for smoother control by interpolating a polynomial curve
		float exponent = 0.72f;
		float multiplier = 1f;

		if (opMode.input.getTrigger(Input.Source.CONTROLLER_1, Input.Button.LEFT_TRIGGER) > 0.4f)
		{
			exponent = 0.56f;
			multiplier = 0.32f;
		}

		positionalInput = positionalInput.normalize().mul((float)Math.pow(positionalInput.getMagnitude(), exponent) * multiplier);
		rotationalInput = Mathf.normalize(rotationalInput) * (float)Math.pow(Math.abs(rotationalInput), exponent) * multiplier;

		//If no rotational input, then IMU is used to counterbalance hardware inaccuracy to drive straight
		if (!positionalInput.equals(Vector2.zero)) angleCorrection = true;
		if (!Mathf.almostEquals(rotationalInput, 0f)) angleCorrection = false;

		if (opMode.input.getTrigger(Input.Source.CONTROLLER_1, Input.Button.RIGHT_TRIGGER) > 0.4f) angleCorrection = false;

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
	}

	public SampleMecanumDrive getDrive()
	{
		return drive;
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

//		opMode.debug.addData("Position", getAveragePosition());
//		opMode.debug.addData("Powers", Arrays.toString(powers));
	}

	private void resetMotorPositions()
	{
		for (DcMotor motor : motors)
		{
			motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		}
	}

	private float getAngle()
	{
		return Mathf.toSignedAngle(imu.getAngles().z);
	}

	@Override
	public void onJobAdded()
	{
		super.onJobAdded();

		Job job = getCurrentJob();

		if (job instanceof Follow)
		{
			Follow follow = (Follow)job;

			drive.followTrajectoryAsync(follow.trajectory);
			drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		}
	}

	@Override
	protected void updateJob()
	{
		Job job = getCurrentJob();

		drive.update();

		if (!drive.isBusy())
		{
			job.finishJob();
			resetMotorPositions();
		}
	}

	static abstract class Job extends FTCEngine.Core.Auto.Job
	{
	}

	public static class Follow extends Job
	{
		public Follow(Trajectory trajectory)
		{
			this.trajectory = trajectory;
		}

		public final Trajectory trajectory;
	}
}
