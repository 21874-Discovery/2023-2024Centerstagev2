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

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name="random auto", group="Robot")
public class RoBoBoTowPoGoAuto extends LinearOpMode {

   // declare OpMode members


   private DcMotor topLeft;
   private DcMotor topRight;
   private DcMotor bottomLeft;
   private DcMotor bottomRight;




   private final double ppr=537.7;
   private final double     dgr    = 1.0 ;     // No External Gearing.
   private final double     wd   = 3.77953 ;     // For figuring circumference
   private final double     tpi         = (ppr * dgr)/(wd * Math.PI);

   @Override
   public void runOpMode() {

      topLeft=hardwareMap.get(DcMotor.class,"topLeft");
      topRight=hardwareMap.get(DcMotor.class,"topRight");
      bottomLeft=hardwareMap.get(DcMotor.class,"bottomLeft");
      bottomRight=hardwareMap.get(DcMotor.class,"bottomRight");


      //zero power behav

      //set position of wanted inches

      topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
      topRight.setDirection(DcMotorSimple.Direction.FORWARD);
      bottomLeft.setDirection(DcMotorSimple.Direction.REVERSE);
      bottomRight.setDirection(DcMotorSimple.Direction.FORWARD);



      topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);






      telemetry.addData("Status", "Initialized");
      telemetry.update();


      // init intake motors
      // init servo

      // Wait for the game to start (driver presses PLAY)
      waitForStart();


      topLeft.setTargetPosition((int)(2900*tpi));
      topRight.setTargetPosition((int)(2900*tpi));
      bottomLeft.setTargetPosition((int)(2900*tpi));
      bottomRight.setTargetPosition((int)(2900*tpi));


      topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      bottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      bottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

      topLeft.setPower(dashboardData.drivePower);
//      topRight.setPower(dashboardData.drivePower);
//      bottomLeft.setPower(dashboardData.drivePower);
//      bottomRight.setPower(dashboardData.drivePower);



      // run until the end of the match (driver presses STOP)

      while (topLeft.isBusy()) {
         /* READ INPUTS */
         // read current values of joystick



         telemetry.addData("topLeft: ", topLeft.getCurrentPosition());
         telemetry.addData("topRight: ", topRight.getCurrentPosition());
         telemetry.addData("bottomLeft: ", bottomLeft.getCurrentPosition());
         telemetry.addData("bottomRightt: ", bottomRight.getCurrentPosition());
         telemetry.update();

      }
      topLeft.setPower(0.0);
      topRight.setPower(0.0);
      bottomLeft.setPower(0.0);
      bottomRight.setPower(0.0);
   }
}