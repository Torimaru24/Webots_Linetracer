// PID制御編 
/*
 * Copyright 1996-2020 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:  An example of use of a camera device.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/camera.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/utils/system.h>

#define ANSI_COLOR_RED "\x1b[31m"
#define ANSI_COLOR_GREEN "\x1b[32m"
#define ANSI_COLOR_YELLOW "\x1b[33m"
#define ANSI_COLOR_BLUE "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN "\x1b[36m"
#define ANSI_COLOR_RESET "\x1b[0m"

enum BLOB_TYPE { RED, GREEN, BLUE, NONE };
 
int main() {
  WbDeviceTag camera, left_motor, right_motor;
  int width, height;
  int pause_counter = 0;
  int left_speed = 0, right_speed = 0;
  int i, j, k, l;
  const int SPEED = 3;
  const double Kp = 0.00015 * SPEED, Ki = 0.00002 * SPEED, Kd = 0.0002 * SPEED;
  int intensity[5];
  double error[3]; error[0] = 0, error[1] = 0, error[2] = 0;
  double control;

  wb_robot_init();

  const int time_step = wb_robot_get_basic_time_step();

  /* Get the camera device, enable it, and store its width and height */
  camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, time_step);
  width = wb_camera_get_width(camera);
  height = wb_camera_get_height(camera);

  /* get a handler to the motors and set target position to infinity (speed control). */
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  /* Main loop */
  while (wb_robot_step(time_step) != -1) {
    /* Get the new camera values */
    const unsigned char *image = wb_camera_get_image(camera);
    /* Decrement the pause_counter */
    if (pause_counter > 0)
      pause_counter--;

    /*
     ここからライントレースの処理
     
     */
    if (image){  //時々画像が抜ける可能性もあるらしい
      //画像処理。フォトダイオードを再現。
      // 〇〇●〇〇のように取得される。 ○=0、●=255
      for (k = 0; k < 5; k++) {
        intensity[k] = 0;
        l = 0;
        for (i = (5+2*k) * width / 20; i < (7+2*k) * width / 20; i++) {
          for (j = 3* height / 4; j < height; j++) {
            intensity[k] += 255 - wb_camera_image_get_gray(image, width, i, j); //白黒反転
            l++;
          }
        }
        intensity[k] /= l; // 各マスの値を0~255に変換
      }
      
      //飛び出した場合のエラー処理。明らかにすべてが白のときには後ろに下がる。
      if (intensity[0] + intensity[1] + intensity[2] + intensity[3] + intensity[4] < 100) {
        left_speed = -SPEED;
        right_speed = -SPEED;
      } else {
        //PID制御。
        error[0] = error[1]; //error[0] : 1ステップ前の誤差
        error[1] = (2*intensity[4] + intensity[3] - intensity[1] - 2*intensity[0]); //error[1] : 現在の誤差
        error[2] += (error[0] + error[1]) / 2.0;
        control = Kp * error[1] + Ki * error[2] + Kd * (error[1] - error[0]);
        left_speed = SPEED * (1 + control);
        right_speed = SPEED * (1 - control);
      }
    }

    /* Set the motor speeds. */
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  }

  wb_robot_cleanup();

  return 0;
}
