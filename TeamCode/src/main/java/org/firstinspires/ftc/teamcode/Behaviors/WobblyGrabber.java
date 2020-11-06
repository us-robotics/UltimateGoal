package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import FTCEngine.Core.Behavior;
import FTCEngine.Core.Input;
import FTCEngine.Core.OpModeBase;
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

		arm = hardwareMap.dcMotor.get("frontLeft");
		arm.setPower(0d);

		grabber = hardwareMap.servo.get("grabber");
		grabber.setPosition(0);

		touch = hardwareMap.touchSensor.get("touch");

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

		float armInput = (touch.isPressed() ? 0 : input.getVector(Input.Source.CONTROLLER_2, Input.Button.LEFT_JOYSTICK).y);
		boolean grabberInput = input.getButton(Input.Source.CONTROLLER_2, Input.Button.A);

		//Process input for smoother control
		//armInput = 0.8f * armInput;

		//Set zero power behavior
		boolean hasMovement = !Mathf.almostEquals(armInput, 0f);

		arm.setZeroPowerBehavior(hasMovement ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE);

		arm.setPower(armInput*armInput*0.9f);

		if (grabberInput)
		{
			grabber.setPosition(0.0f);
		}
		else grabber.setPosition(0.8f);
	}
}
