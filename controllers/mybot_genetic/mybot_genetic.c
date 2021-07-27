// 遺伝的アルゴリズム編。 
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
#include <math.h>
#include <webots/camera.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/receiver.h>
#include <webots/emitter.h>
#include <webots/utils/system.h>

typedef struct {
  double Kp;
  double Ki;
  double Kd;
  bool isDisqualified; //走行中にコースアウトして失格になったか
  double elapsedTime; //失格でない場合：周回にかかった時間、失格だった場合：走れた時間
  int rank; //同一世代内での上から数えたランク。
} myGA_data; 
 
int main() {
  bool END = false; //100世代終わったらすべて止める。
  WbDeviceTag camera, left_motor, right_motor, receiver, emitter;
  int width, height;
  int pause_counter = 0;
  double left_speed = 0, right_speed = 0;
  int i, j, k, l;
  
  const int individuals = 12; //1世代の個体数。GAの都合で必ず3以上。
  const int generations = 100; //遺伝的アルゴリズムを回す世代数
  myGA_data mydata[individuals];
  int now_individual = 0; //0 ~ (individuals-1) まで
  int now_generation = 0; //0 ~ (generation-1) まで
  double time_history[generations];
  
  const int SPEED = 3; //スピードを上げる勝負になってしまうからこれは固定。
  //あと、スピードを上げるとなんかロボットが浮いて壊れちゃう
  
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

  /* Get the receiver */
  receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, time_step);
  emitter = wb_robot_get_device("emitter");
  bool packet[1]; //1個体の終わり(ゴールについた、失格した)をsupervisorに伝えるための信号。
  
  
  //まず、適当にspeed、P、I、Dを決め、15個体を用意する。
  for(int a = 0; a < individuals; a++){
    mydata[a].Kp = ((double)rand()/RAND_MAX) * 0.0006 + 0.0003;  //0.0001 ~ 0.01くらい
    mydata[a].Ki = ((double)rand()/RAND_MAX) * 0.0001;
    mydata[a].Kd = ((double)rand()/RAND_MAX) * 0.001;
    //mydata[a].Kp = 0.0393, mydata[a].Ki = 0.044, mydata[a].Kd = 0.055; //学習後
    
    mydata[a].isDisqualified = false;
    mydata[a].elapsedTime = 0;
    mydata[a].rank = 0;
  }
  
  /* Main loop */
  while (wb_robot_step(time_step) != -1) {
    if(END) break;
    //supervisorからの司令を受ける。来るデータは「前周回の時間」。
    //これが来るときには必ず座標が初期化されて、次の周回が始まる。
    while (wb_receiver_get_queue_length(receiver) > 0) {
      const double *message = wb_receiver_get_data(receiver);
      //前の周回にかかった時間(もしくは失格までに走れた時間)が乗ってるので、記録する。
      mydata[now_individual].elapsedTime =  message[0]; 
      //ちょっと怪しい挙動が一部にあったので
      if(mydata[now_individual].elapsedTime < 1) mydata[now_individual].isDisqualified = true;
      
      if (mydata[now_individual].isDisqualified){
        printf("個体%d： 時間%g秒、失格\n", now_individual, mydata[now_individual].elapsedTime);
      }else{
        printf("個体%d： 時間%g秒\n", now_individual, mydata[now_individual].elapsedTime);
      }
      //次の周回の準備。
      now_individual++;
      error[0] = 0; //問題になるのはIだけだが、条件を同じにするために全部リセット。
      error[1] = 0;
      error[2] = 0; //これ大事。Iの累積をリセットする。
      wb_motor_set_position(left_motor, INFINITY);
      wb_motor_set_position(right_motor, INFINITY);
      wb_motor_set_velocity(left_motor, 0.0);
      wb_motor_set_velocity(right_motor, 0.0);
  
      wb_receiver_next_packet(receiver);
    }
    
    //1世代が終わった場合、次の世代を準備する。ここ遺伝的アルゴリズム。
    if (now_individual == individuals){
      //まず、すべての結果を比較し、ランキングを作る。バブルソート。
      myGA_data temp;
      myGA_data newdata[individuals];
      int DisQualifiedLimit = 0;
      for(int a = 0; a < individuals; a++){
        for(int b = a+1; b < individuals; b++){
          //ランキングの順番は、先頭(a)のほうが優秀に並べるべきで、
          //[最速,合格] > [最遅,合格] > [最長,失格] > [最短,失格]
          if(!mydata[a].isDisqualified && !mydata[b].isDisqualified){//どちらも合格
            if(mydata[a].elapsedTime > mydata[b].elapsedTime){
              temp = mydata[a];
              mydata[a] = mydata[b];
              mydata[b] = temp;
            }
          }else if(!mydata[a].isDisqualified && mydata[b].isDisqualified){//bのみ失格
            // pass
          }else if(mydata[a].isDisqualified && !mydata[b].isDisqualified){//aのみ失格
              temp = mydata[a];
              mydata[a] = mydata[b];
              mydata[b] = temp;
          }else{ //どちらも失格
            if(mydata[a].elapsedTime < mydata[b].elapsedTime){
              temp = mydata[a];
              mydata[a] = mydata[b];
              mydata[b] = temp;
            }
          }
        }
      }
      //ゴールできた個体の範囲を取得する。
      for(int a = 0; a < individuals; a++){
        if(mydata[a].isDisqualified){
          DisQualifiedLimit = a;
          break;
        }
      }
      printf("第 %d 世代が終了しました。最速：%g秒\n", now_generation, mydata[0].elapsedTime);
      time_history[now_generation] = mydata[0].elapsedTime;
      //上位3個体はそのままパラメータを採用。
      newdata[0] = mydata[0];
      newdata[1] = mydata[1];
      newdata[2] = mydata[2];
      //下位12個体は、ゴールできた個体の中から2個体を選択し、ランダムに内分する。
      if(DisQualifiedLimit < 2){
        //ゴールできた個体が0個か1個のとき
        //問題発生なので、ある程度妥当な範囲で埋める
        for(int a = 3; a < individuals; a++){
          newdata[a].Kp = ((double)rand()/RAND_MAX) * 0.0006 + 0.0003; //0.0001 ~ 0.01くらい
          newdata[a].Ki = ((double)rand()/RAND_MAX) * 0.0001;
          newdata[a].Kd = ((double)rand()/RAND_MAX) * 0.001;
        }
      }else{
        //ゴールできた個体が2個以上あるとき、2個体を選択し、ランダムに内分する。
        for(int a = 3; a < individuals; a++){
          int t1 = rand() % DisQualifiedLimit;
          int t2 = rand() % DisQualifiedLimit;
          newdata[a].Kp = mydata[t1].Kp + ((double)rand()/RAND_MAX) * (mydata[t2].Kp - mydata[t1].Kp);
          newdata[a].Ki = mydata[t1].Ki + ((double)rand()/RAND_MAX) * (mydata[t2].Ki - mydata[t1].Ki);
          newdata[a].Kd = mydata[t1].Kd + ((double)rand()/RAND_MAX) * (mydata[t2].Kd - mydata[t1].Kd);
        }
      }
      //下の方の3個体は、突然変異を食わせる。最下位まで含めると、なぜか１位に時々バグを起こすので、下から4位〜2位。
      for(int a = individuals - 4; a < individuals - 1; a++){
        newdata[a].Kp += ((double)rand()/RAND_MAX ) * 0.01; //-0.005 ~ 0.005を足す。
        newdata[a].Ki += ((double)rand()/RAND_MAX ) * 0.01;
        newdata[a].Kd += ((double)rand()/RAND_MAX ) * 0.01;
      }
      
      
      
      //各個体のパラメータを再設定して次の世代にする。
      for(int a = 0; a < individuals; a++){
        mydata[a].Kp = newdata[a].Kp;
        mydata[a].Ki = newdata[a].Ki;
        mydata[a].Kd = newdata[a].Kd;
        mydata[a].isDisqualified = false;
        mydata[a].elapsedTime = 0;
        mydata[a].rank = 0;
      }
      
      now_individual = 0;
      now_generation++;
      if(now_generation > generations){
        //本来は更新は終了。ただ、別に続けても問題ない気もする。
        printf("決められた世代数が経過しました。\n");
        printf("１位：%g秒、Kp：%g、Ki：%g、Kd：%g\n", newdata[0].elapsedTime, newdata[0].Kp, newdata[0].Ki, newdata[0].Kd);
        printf("２位：%g秒、Kp：%g、Ki：%g、Kd：%g\n", newdata[1].elapsedTime, newdata[1].Kp, newdata[1].Ki, newdata[1].Kd);
        printf("３位：%g秒、Kp：%g、Ki：%g、Kd：%g\n", newdata[2].elapsedTime, newdata[2].Kp, newdata[2].Ki, newdata[2].Kd);
        for(int a = 0; a < generations; a++){
          printf("%d, %g\n", a, time_history[a]);
        }
        END = true;
      }
    }
    
    /* Get the new camera values */
    const unsigned char *image = wb_camera_get_image(camera);
    /* Decrement the pause_counter */
    if (pause_counter > 0)
      pause_counter--;

    /*
     ここからライントレースの処理。
     
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
    } 
      
     //保険。
      left_speed = 0;
      right_speed = 0;
    //こちらの場合、飛び出した場合は失格とする。
    if (intensity[0] + intensity[1] + intensity[2] + intensity[3] + intensity[4] < 100) {
      mydata[now_individual].isDisqualified = true;
       //supervisorに知らせる。
       packet[0] = true;
       wb_emitter_send(emitter, packet, sizeof(packet));
      left_speed = 0;
      right_speed = 0;
    } else {
      //PID制御する。
      error[0] = error[1]; //error[0] : 1ステップ前の誤差
      error[1] = (2*intensity[4] + intensity[3] - intensity[1] - 2*intensity[0]); //error[1] : 現在の誤差
      error[2] += (error[0] + error[1]) / 2.0;
      control = mydata[now_individual].Kp * error[1] + mydata[now_individual].Ki * error[2] + mydata[now_individual].Kd * (error[1] - error[0]);
      
      left_speed = SPEED * (1 + control);
      right_speed = SPEED * (1 - control);
      if(left_speed > 10) left_speed = 10;
      if(right_speed > 10) right_speed = 10;
      if(left_speed < 0) left_speed = 0;
      if(right_speed < 0) right_speed = 0;
    }
    
    //もし一周することに成功したら、それをsupervisorに知らせる。
    // ......予定であったが、座標が把握できないため、supervisorにその判断を任せる。
      
   
    /* Set the motor speeds. */
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  }

  wb_robot_cleanup();

  return 0;
}
