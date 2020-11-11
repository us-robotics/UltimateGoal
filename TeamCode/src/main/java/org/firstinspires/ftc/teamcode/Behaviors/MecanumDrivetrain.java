package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import FTCEngine.Core.Behavior;
import FTCEngine.Core.Input;
import FTCEngine.Core.OpModeBase;
import FTCEngine.Math.Mathf;
import FTCEngine.Math.Vector2;
import FTCEngine.Math.Vector3;

public class MecanumDrivetrain extends Behavior
{
	/**
	 * NOTE: Do not configure the electronics in the constructor, do them in the awake method!
	 */
	public MecanumDrivetrain(OpModeBase opMode)
	{
		super(opMode);
	}

	@Override
	public void awake(HardwareMap hardwareMap)
	{
		super.awake(hardwareMap);

		frontLeft = hardwareMap.dcMotor.get("frontLeft");
		frontRight = hardwareMap.dcMotor.get("frontRight");
		backLeft = hardwareMap.dcMotor.get("backLeft");
		backRight = hardwareMap.dcMotor.get("backRight");

		frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
		backRight.setDirection(DcMotorSimple.Direction.REVERSE);

		imu = opMode.getBehavior(InternalMeasurementUnit.class);
		setRawVelocities(Vector2.zero, 0f);
	}

	private DcMotor frontRight;
	private DcMotor frontLeft;
	private DcMotor backRight;
	private DcMotor backLeft;

	private InternalMeasurementUnit imu;
	private float lastAngle;

	private final float[] powers = new float[4];

	@Override
	public void start()
	{
		super.start();

		if (imu == null) return;
		lastAngle = getZAngle();
	}

	@Override
	public void update()
	{
		super.update();

		if (getIsAuto()) return; //Auto will be controlled through another behavior
		Input input = opMode.getHelper(Input.class);

		Vector2 positionalInput = input.getVector(Input.Source.CONTROLLER_1, Input.Button.LEFT_JOYSTICK);
		float rotationalInput = input.getVector(Input.Source.CONTROLLER_1, Input.Button.RIGHT_JOYSTICK).x;

		//Process input for smoother control by interpolating a polynomial curve
		final float exponential = 2f; //Can use higher power if more precision is needed

		positionalInput = positionalInput.normalize().mul((float)Math.pow(positionalInput.getMagnitude(), exponential));
		rotationalInput = Mathf.normalize(rotationalInput) * (float)Math.pow(Math.abs(rotationalInput), exponential);

		setMovements(positionalInput, rotationalInput);
	}

	private void setMovements(Vector2 positionalMovement, float rotationalMovement)
	{
		//If no rotational input, then IMU is used to counterbalance hardware inaccuracy to drive straight
		if (imu != null && Mathf.almostEquals(rotationalMovement, 0f))
		{
			float deviation = getZAngle() - lastAngle;
			setRawVelocities(positionalMovement, deviation);
		}
		else
		{
			setRawVelocities(positionalMovement, rotationalMovement);
			if (imu != null) lastAngle = getZAngle();
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
			max = Math.max(max, power);
		}

		if (max > 1f)
		{
			//Scales all powers down by a multiplier if one power is higher than 1
			for (int i = 0; i < powers.length; i++) powers[i] /= max;
		}

		boolean hasPower = !Mathf.almostEquals(max, 0f); //Switches between the two zero power behaviors to brake
		setZeroPowerBehavior(hasPower ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE);

		frontRight.setPower(powers[0]);
		frontLeft.setPower(powers[1]);
		backRight.setPower(powers[2]);
		backLeft.setPower(powers[3]);
	}

	private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) //what does this do?
	{
		frontRight.setZeroPowerBehavior(behavior);
		frontLeft.setZeroPowerBehavior(behavior);
		backRight.setZeroPowerBehavior(behavior);
		backLeft.setZeroPowerBehavior(behavior);
	}

	private float getZAngle()
	{
		return imu.getAngles().z;
	}
}
