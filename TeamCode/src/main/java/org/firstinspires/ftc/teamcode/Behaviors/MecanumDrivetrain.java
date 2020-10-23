package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import FTCEngine.Core.Behavior;
import FTCEngine.Core.Input;
import FTCEngine.Core.OpModeBase;
import FTCEngine.Math.Mathf;
import FTCEngine.Math.Vector2;

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

		frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
		backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

		frontLeft.setPower(0d);
		frontRight.setPower(0d);
		backLeft.setPower(0d);
		backRight.setPower(0d);
	}

	private DcMotor frontRight;
	private DcMotor frontLeft;
	private DcMotor backRight;
	private DcMotor backLeft;

	@Override
	public void update()
	{
		super.update();
		Input input = opMode.getHelper(Input.class);

		Vector2 movementInput = input.getVector(Input.Source.CONTROLLER_1, Input.Button.LEFT_JOYSTICK).normalize();
		float rotationInput = input.getVector(Input.Source.CONTROLLER_1, Input.Button.RIGHT_JOYSTICK).x;

		//Process input for smoother control
		movementInput = movementInput.mul(movementInput.getMagnitude());
		rotationInput *= Math.abs(rotationInput);

		Vector2 velocity = movementInput;
		float angularVelocity = rotationInput;

		//Set zero power behavior
		boolean hasMovement = !velocity.equals(Vector2.zero) || !Mathf.almostEquals(angularVelocity, 0f);
		setZeroPowerBehavior(hasMovement ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE);

		frontRight.setPower(-velocity.y + velocity.x + angularVelocity);
		frontLeft.setPower(-velocity.y - velocity.x - angularVelocity);
		backRight.setPower(-velocity.y - velocity.x + angularVelocity);
		backLeft.setPower(-velocity.y + velocity.x - angularVelocity);
	}

	private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior)
	{
		frontRight.setZeroPowerBehavior(behavior);
		frontLeft.setZeroPowerBehavior(behavior);
		backRight.setZeroPowerBehavior(behavior);
		backLeft.setZeroPowerBehavior(behavior);
	}
}
