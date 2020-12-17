package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import FTCEngine.Core.Behavior;
import FTCEngine.Core.Input;
import FTCEngine.Core.OpModeBase;
import FTCEngine.Math.Mathf;

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
		launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		trigger.setPosition(0.435d);

		opMode.getHelper(Input.class).registerButton(Input.Source.CONTROLLER_2, Input.Button.LEFT_BUMPER);
		opMode.getHelper(Input.class).registerButton(Input.Source.CONTROLLER_2, Input.Button.RIGHT_BUMPER);
	}

	private DcMotor launcher;
	private Servo trigger;

	private boolean primed;

	@Override
	public void update()
	{
		super.update();

		if (opMode.getIsAuto()) return;
		Input input = opMode.getHelper(Input.class);

		if (input.getButtonDown(Input.Source.CONTROLLER_2, Input.Button.LEFT_BUMPER)) primed = !primed;

		boolean pressed = input.getButton(Input.Source.CONTROLLER_2, Input.Button.RIGHT_BUMPER);
		boolean reduced = !Mathf.almostEquals(input.getTrigger(Input.Source.CONTROLLER_2, Input.Button.LEFT_TRIGGER), 0);

		launcher.setPower(primed ? (reduced ? 0.2d : 0.8d) : 0d);
		trigger.setPosition(pressed ? 0.1d : 0.435d);
	}
}
