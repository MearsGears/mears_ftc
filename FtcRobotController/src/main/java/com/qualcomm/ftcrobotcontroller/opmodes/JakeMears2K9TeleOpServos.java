/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


public class JakeMears2K9TeleOpServos extends OpMode {

	/*
 * Note: the configuration of the servos is such that
 * as the armservo servo approaches 0, the armservo position moves up (away from the floor).
 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
 */
	// TETRIX VALUES.
	final static double armservo_MIN_RANGE  = 0.20;
	final static double armservo_MAX_RANGE  = 0.90;
	final static double CLAW_MIN_RANGE  = 0.20;
	final static double CLAW_MAX_RANGE  = 0.90;

	// position of the armservo servo.

	double armservoPosition;

	// amount to change the armservo servo position.
	double armservoDelta = 0.2;

	// position of the claw servo
	double clawPosition;

	// amount to change the claw servo position by
	double clawDelta = 0.2;


	Servo claw;
	Servo armservo;
	DcMotor arm_extend;
	DcMotor leftMotor;
	DcMotor rightMotor;
	DcMotor leftArm;

	@Override
	public void init() {
		//get references to the motors from hardware map
		leftMotor = hardwareMap.dcMotor.get("left_drive");
		rightMotor = hardwareMap.dcMotor.get("right_drive");
		leftArm = hardwareMap.dcMotor.get("left_arm");
		arm_extend = hardwareMap.dcMotor.get("arm_drive");
		armservo = hardwareMap.servo.get("servo_1");
		claw = hardwareMap.servo.get("servo_6");

		// assign the starting position of the wrist and claw
		armservoPosition = 0.2;
		clawPosition = 0.2;

		//reverse the left motor
		rightMotor.setDirection(DcMotor.Direction.REVERSE);

	}

	@Override
	public void loop() {
		//get values from gamepads
		//note: pushing the stick all the way up returns -1, so we need to reverse the values
		float leftY = -gamepad1.left_stick_y;
		float rightY = -gamepad1.right_stick_y;
		float Arm = -gamepad2.left_stick_y;
		float extend = -gamepad2.right_stick_y;

		//set the power of the motors with the gamepad values
		leftMotor.setPower(leftY);
		rightMotor.setPower(rightY);
		leftArm.setPower(Arm);
		arm_extend.setPower(extend);

		// update the position of the armservo.
		if (gamepad2.b) {
			// if the A button is pushed on gamepad1, increment the position of
			// the armservo servo.
			armservoPosition += armservoDelta;
		}

		if (gamepad2.x) {
			// if the Y button is pushed on gamepad1, decrease the position of
			// the armservo servo.
			armservoPosition -= armservoDelta;
		}

		// update the position of the claw
		if (gamepad2.x) {
			clawPosition += clawDelta;
		}

		if (gamepad2.b) {
			clawPosition -= clawDelta;
		}

		// clip the position values so that they never exceed their allowed range. hello
		armservoPosition = Range.clip(armservoPosition, armservo_MIN_RANGE, armservo_MAX_RANGE);
		clawPosition = Range.clip(clawPosition, CLAW_MIN_RANGE, CLAW_MAX_RANGE);

		// write position values to the wrist and claw servo
		armservo.setPosition(armservoPosition);
		claw.setPosition(clawPosition);



		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
		telemetry.addData("Text", "*** Robot Data***");
		telemetry.addData("armservo", "armservo:  " + String.format("%.2f", armservoPosition));
		telemetry.addData("claw", "claw:  " + String.format("%.2f", clawPosition));

	}

	/*
	 * Code to run when the op mode is first disabled goes here
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
	@Override
	public void stop() {

	}


	/*
	 * This method scales the joystick input so for low joystick values, the
	 * scaled value is less than linear.  This is to make it easier to drive
	 * the robot more precisely at slower speeds.
	 */
	double scaleInput(double dVal)  {
		double[] scaleArray = { 0.0, 0.009, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.10, 0.12, 0.14, 0.15, 0.18, 0.19, 0.21, 0.22, 0.24, 0.25, 0.28,
				0.30, 0.36, 0.43, 0.47, 0.50, 0.55, 0.60, 0.66, 0.72, 0.85, 1.00, 2.00, 3.00 };

		// get the corresponding index for the scaleInput array.
		int index = (int) (dVal * 16.0);

		// index should be positive.
		if (index < 0) {
			index = -index;
		}

		// index cannot exceed size of array minus 1.
		if (index > 16) {
			index = 16;
		}


		// get value from the array.
		double dScale = 0.0;
		if (dVal < 0) {
			dScale = -scaleArray[index];
		} else {
			dScale = scaleArray[index];
		}

		// return scaled value.
		return dScale;
	}

}







