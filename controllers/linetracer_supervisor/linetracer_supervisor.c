// ライントレーサ用のスーパーバイザ。
// まずはR3 (強化学習) の初期化だけを担当。
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
 * Description:  エージェントシステム用改造
                 Supevisor the soccer game from soccer.wbt
 *               Send the coordinates and orientations of each robot and the
 *               coordinates of the ball to each robot via an emitter.
 */

#include <stdio.h>
#include <string.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#define ROBOTS 1  // number of robots

/* 
static void set_scores(int b, int y) {
  char score[16];

  sprintf(score, "%d", b);
  wb_supervisor_set_label(0, score, 0.05, 0.01, 0.1, 0x0000ff, 0.0, "Arial");  // blue
  sprintf(score, "%d", y);
  wb_supervisor_set_label(1, score, 0.92, 0.01, 0.1, 0xff6666, 0.0, "Arial");  // red
}
*/

int main() {
  const char *robot_name[ROBOTS] = {"R4"};
  WbNodeRef node;
  WbFieldRef robot_translation_field[ROBOTS], robot_rotation_field[ROBOTS];
  WbDeviceTag emitter;
  WbDeviceTag receiver;
  int i, j;
  
  double robot_initial_translation[ROBOTS][3] = {{0.15, 0, -0.38}};
  double robot_initial_rotation[ROBOTS][4] = {{0, 1, 0, 1.5708}};
  const double *robot_translation[ROBOTS], *robot_rotation[ROBOTS];

  wb_robot_init();
  const int time_step = wb_robot_get_basic_time_step();

  receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, time_step);
  emitter = wb_robot_get_device("emitter");
  double thisTime = 0; //前周回にかかった時間。開始は常に0。
  double packet[1]; //送信用バッファ

  //初期化用の座標と角度を記録。
  for (i = 0; i < ROBOTS; i++) {
    node = wb_supervisor_node_get_from_def(robot_name[i]);
    robot_translation_field[i] = wb_supervisor_node_get_field(node, "translation");
    robot_translation[i] = wb_supervisor_field_get_sf_vec3f(robot_translation_field[i]);
    robot_rotation_field[i] = wb_supervisor_node_get_field(node, "rotation");
    robot_rotation[i] = wb_supervisor_field_get_sf_rotation(robot_rotation_field[i]);
    for (j = 0; j < 3; j++){
      robot_initial_translation[i][j] = robot_translation[i][j];
    }
    for (j = 0; j < 4; j++){
      robot_initial_rotation[i][j] = robot_rotation[i][j];
    }
  }
  const double inix = 0.95;
  const double iniz = 0.69; //直打ちだけど許してくれ

  //スーパバイザの役割。スーパバイザは時刻を返して初期化を手伝う。
  //さらに、周回に成功したかどうかもこちらで判断する。
  //遺伝的アルゴリズムの本体はすべてロボット内に実装している。
  while (wb_robot_step(time_step) != -1) { 
    //ロボットの周回成功を監視する。
    //初期座標より少し手前(=回ってこないと到達できない位置)とのズレが十分小さいことで判断する。
    // ※初期translation 0.85 0 0.62
    // ※判断translation 0.95 0 0.69
    for (i = 0; i < ROBOTS; i++) {
      robot_translation[i] = wb_supervisor_field_get_sf_vec3f(robot_translation_field[i]);
    }
    double nowx = robot_translation[0][0];  // robot i: X。ロボットが一人仕様なのでちょっと問題あり。
    double nowz = robot_translation[0][2];  // robot i: Z
    // 目標地点に十分に近づいていたら、ロボットの事情にお構いなしに初期化、次の個体の周回を強制する。
    if (sqrt((nowz-iniz) * (nowz-iniz) + (nowx-inix) * (nowx-inix))  < 0.03){
      //ロボットの座標を初期化する。(supervisor特権)
      for (i = 0; i < ROBOTS; i++) {
        wb_supervisor_field_set_sf_vec3f(robot_translation_field[i], robot_initial_translation[i]);
        wb_supervisor_field_set_sf_rotation(robot_rotation_field[i], robot_initial_rotation[i]);
      }
      wb_supervisor_simulation_reset_physics();
      //周回にかかった時間を送信する。
      packet[0] = thisTime;
      wb_emitter_send(emitter, packet, sizeof(packet));
      
      //次の周回の時刻計測を始める。
      thisTime = 0;
    } 
    
    //ロボットから初期化要請が来たら、初期化して現在の時刻を送り返す。
    while (wb_receiver_get_queue_length(receiver) > 0) {
      const bool *message = wb_receiver_get_data(receiver); //このメッセージ自体に用はない。
      //ロボットの座標を初期化する。(supervisor特権)
      for (i = 0; i < ROBOTS; i++) {
        wb_supervisor_field_set_sf_vec3f(robot_translation_field[i], robot_initial_translation[i]);
        wb_supervisor_field_set_sf_rotation(robot_rotation_field[i], robot_initial_rotation[i]);
      }
      wb_supervisor_simulation_reset_physics();
      //周回にかかった時間を送信する。
      packet[0] = thisTime;
      wb_emitter_send(emitter, packet, sizeof(packet));
      
      //次の周回の時刻計測を始める。
      thisTime = 0;
      
      wb_receiver_next_packet(receiver);
    }
    
    thisTime += (double)time_step / 1000; //時刻を計測していく。
  }

  wb_robot_cleanup();

  return 0;
}
