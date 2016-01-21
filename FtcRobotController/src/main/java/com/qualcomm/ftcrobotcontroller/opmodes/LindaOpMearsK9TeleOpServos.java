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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class LindaOpMearsK9TeleOpServos extends LinearOpMode {

	DcMotor leftR;
	DcMotor rightR;
	DcMotor leftMotor;
	DcMotor rightMotor;
	Servo leftS;
	Servo rightS;

	@Override
	public void runOpMode() throws InterruptedException {

		leftMotor = hardwareMap.dcMotor.get("left_motor");
		rightMotor = hardwareMap.dcMotor.get("right_motor");

		rightMotor.setDirection(DcMotor.Direction.REVERSE);


		waitForStart();


		leftMotor.setPower(0.5);
		rightMotor.setPower(0.5);

		sleep(5000);

		leftMotor.setPower(0.5);
		rightMotor.setPower(-0.5);

		sleep(1200);

		leftMotor.setPower(0);
		rightMotor.setPower(0);
	}



	final double LEFT_OPEN_POSITION = 0.0;
	final double LEFT_CLOSED_POSITION = 0.5;
	final double RIGHT_OPEN_POSITION = 1.0;
	final double RIGHT_CLOSED_POSITION = 0.5;


	@Override
	public void init() {

		leftMotor = hardwareMap.dcMotor.get("left_motor");
		rightMotor = hardwareMap.dcMotor.get("right_motor");
		leftS = hardwareMap.servo.get("leftS");
		rightS = hardwareMap.servo.get("rightS");
		leftR = hardwareMap.dcMotor.get("leftR");
		rightR = hardwareMap.dcMotor.get("rightR");

		rightMotor.setDirection(DcMotor.Direction.REVERSE);

	}


	@Override
	public void loop() {

		float leftY = -gamepad1.left_stick_y;
		float rightY = -gamepad1.right_stick_y;

		float rightX = gamepad2.left_stick_y;
		float leftX = gamepad2.left_stick_y;

		leftMotor.setPower(leftY);
		rightMotor.setPower(rightY);

		rightR.setPower(rightX);
		leftR.setPower(leftX);



		if (gamepad2.left_bumper) {
			leftS.setPosition(LEFT_OPEN_POSITION);
			rightS.setPosition(RIGHT_OPEN_POSITION);
		}
		if(gamepad2.right_bumper) {
			leftS.setPosition(LEFT_CLOSED_POSITION);
			rightS.setPosition(RIGHT_CLOSED_POSITION);
		}
	}
}
