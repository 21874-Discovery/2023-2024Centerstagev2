/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="winch", group="Linear Opmode")
public class winch extends LinearOpMode {

   // declare OpMode members


   // dawinching
   private DcMotor winchMotor = null;

   private double winchPower = 0.5;

   @Override
   public void runOpMode() {

      winchMotor = hardwareMap.get(DcMotor.class, "winchMotor");

      telemetry.addData("Status", "Initialized");
      telemetry.update();
      // init intake motors
      // init servo

      // Wait for the game to start (driver presses PLAY)
      waitForStart();

      // run until the end of the match (driver presses STOP)

      while (opModeIsActive()) {
         /* READ INPUTS */
         // read current values of joystick
         float rTrigger = gamepad1.right_trigger;
         float lTrigger = gamepad1.left_trigger;
         winchMotor.setPower(0.0);

         if (rTrigger >= 0.05) {
            winchMotor.setPower(rTrigger);
         }
         else if (lTrigger >= 0.05) {
            winchMotor.setPower(-(lTrigger));
         }
         if (rTrigger >= 0.05 && lTrigger >= 0.05) {
            winchMotor.setPower(rTrigger-lTrigger);
         }

      }
      winchMotor.setPower(0.0);
   }
}