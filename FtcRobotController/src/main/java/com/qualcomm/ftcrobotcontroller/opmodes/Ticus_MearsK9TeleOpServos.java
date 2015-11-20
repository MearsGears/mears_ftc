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
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;



public class Ticus_MearsK9TeleOpServos extends OpMode {


	DcMotor leftMotor;
	DcMotor rightMotor;
	DcMotor leftArm;

	@Override
	public void init() {
		//get references to the motors from hardware map
		leftMotor = hardwareMap.dcMotor.get("left_drive");
		rightMotor = hardwareMap.dcMotor.get("right_drive");
		leftArm = hardwareMap.dcMotor.get("left_arm");

		//reverse the left motor
		rightMotor.setDirection(DcMotor.Direction.REVERSE);


	}

	@Override
	public void loop() {
		//get values from gamepads
		//note: pushing the stick all the way up returns -1, so we need to reverse the values
		float leftY = -gamepad1.left_stick_y;
		float rightY = -gamepad1.right_stick_y;

		//set the power of the motors with the gamepad values
		leftMotor.setPower(leftY);
		rightMotor.setPower(rightY);

		if (gamepad2.y) {
			leftArm.setPower(0.7);
		} else if (gamepad2.a) {
			leftArm.setPower(-0.1);
		} else {
			leftArm.setPower(0);
		}


	}

}
