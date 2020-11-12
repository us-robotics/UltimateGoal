package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import FTCEngine.Core.Behavior;
import FTCEngine.Core.Input;
import FTCEngine.Core.OpModeBase;
import FTCEngine.Core.Telemetry;
import FTCEngine.Math.Mathf;
import FTCEngine.Math.Vector2;

public class WobblyGrabber extends Behavior
{
	public WobblyGrabber(OpModeBase opMode)
	{
		super(opMode);
	}

	public void awake(HardwareMap hardwareMap)
	{
		super.awake(hardwareMap);

		arm = hardwareMap.dcMotor.get("arm");
		grabber = hardwareMap.servo.get("grabber");
		touch = hardwareMap.touchSensor.get("touch");

		arm.setPower(0d);
		arm.setDirection(DcMotorSimple.Direction.REVERSE);
		grabber.setPosition(0);

		opMode.getHelper(Input.class).registerButton(Input.Source.CONTROLLER_2, Input.Button.A);
		arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	}

	private DcMotor arm;
	private Servo grabber;
	private TouchSensor touch;

	public void update()
	{
		super.update();
		Input input = opMode.getHelper(Input.class);

		float armInput = input.getVector(Input.Source.CONTROLLER_2, Input.Button.LEFT_JOYSTICK).y;
		boolean grabberInput = input.getButton(Input.Source.CONTROLLER_2, Input.Button.A);

		//Process input for smoother/better control
		if (touch.getValue() > 0.2d && armInput > 0f) armInput = 0f;
		armInput = Mathf.normalize(armInput) * (float)Math.abs(Math.pow(armInput, 3f));

		//Set zero power behavior
		arm.setZeroPowerBehavior(Mathf.almostEquals(armInput, 0f) ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);

		arm.setPower(armInput);
		grabber.setPosition(grabberInput ? 1f : 0f);
	}
}
