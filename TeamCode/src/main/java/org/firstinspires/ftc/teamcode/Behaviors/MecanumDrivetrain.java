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

		launcher = hardwareMap.dcMotor.get("launcher");

		launcher.setPower(0d);
	}

	private DcMotor frontRight;
	private DcMotor frontLeft;
	private DcMotor backRight;
	private DcMotor backLeft;

	private DcMotor launcher;

	@Override
	public void update()
	{
		super.update();
		Input input = opMode.getHelper(Input.class);

		Vector2 movementInput = input.getVector(Input.Source.CONTROLLER_1, Input.Button.LEFT_JOYSTICK);
		float rotationInput = input.getVector(Input.Source.CONTROLLER_1, Input.Button.RIGHT_JOYSTICK).x;

		//Process input for smoother contro
		movementInput = movementInput.normalize().mul(movementInput.getMagnitude());
		rotationInput *= Math.abs(rotationInput);

		//Set zero power behavior
		boolean hasMovement = !movementInput.equals(Vector2.zero) || !Mathf.almostEquals(rotationInput, 0f);
		setZeroPowerBehavior(hasMovement ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE);

		boolean launcherInput = input.getButtonDown(Input.Source.CONTROLLER_1, Input.Button.A);

		frontRight.setPower(-movementInput.y + movementInput.x + rotationInput);
		frontLeft.setPower(-movementInput.y - movementInput.x - rotationInput);
		backRight.setPower(-movementInput.y - movementInput.x + rotationInput);
		backLeft.setPower(-movementInput.y + movementInput.x - rotationInput);

		if (launcherInput){
			launcher.setPower(1d); //double btwn -1.0 and 1.0
		}
	}

	private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) //what does this do?
	{
		frontRight.setZeroPowerBehavior(behavior);
		frontLeft.setZeroPowerBehavior(behavior);
		backRight.setZeroPowerBehavior(behavior);
		backLeft.setZeroPowerBehavior(behavior);
		launcher.setZeroPowerBehavior(behavior);
	}
}
