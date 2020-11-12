package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import FTCEngine.Core.Behavior;
import FTCEngine.Core.Input;
import FTCEngine.Core.OpModeBase;

public class Launcher extends Behavior
{
	/**
	 * NOTE: Do not configure the electronics in the constructor, do them in the awake method!
	 *
	 * @param opMode
	 */
	public Launcher(OpModeBase opMode)
	{
		super(opMode);
	}

	@Override
	public void awake(HardwareMap hardwareMap)
	{
		super.awake(hardwareMap);

		launcher = hardwareMap.dcMotor.get("launcher");
		trigger = hardwareMap.servo.get("trigger");

		launcher.setPower(0d);
		trigger.setPosition(0d);

		opMode.getHelper(Input.class).registerButton(Input.Source.CONTROLLER_2, Input.Button.X);
		opMode.getHelper(Input.class).registerButton(Input.Source.CONTROLLER_2, Input.Button.Y);
	}

	private DcMotor launcher;
	private Servo trigger;

	private boolean primed;

	@Override
	public void update()
	{
		super.update();

		Input input = opMode.getHelper(Input.class);

		if (input.getButtonDown(Input.Source.CONTROLLER_2, Input.Button.X)) primed = !primed;
		boolean pressed = input.getButton(Input.Source.CONTROLLER_2, Input.Button.Y);

		launcher.setPower(primed ? 1d : 0d);
		trigger.setPosition(pressed ? 1d : 0d);
	}
}
