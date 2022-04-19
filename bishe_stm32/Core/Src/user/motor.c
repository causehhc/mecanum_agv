//
// Created by hhc on 2022/4/15.
//

#include "user/motor.h"

void set_ENC(motorInfoType *motorInfo){
  motorInfo->pidInfo.ENC = (int16_t)__HAL_TIM_GET_COUNTER(&motorInfo->enc_htimx);
  __HAL_TIM_SET_COUNTER(&motorInfo->enc_htimx, 0);
}

void set_ADD(motorInfoType *motorInfo){
  motorInfo->pidInfo.ADD += motorInfo->pidInfo.ENC;
}

void incremental_PI(motorInfoType *motorInfo){
  motorInfo->pidInfo.Bias = motorInfo->pidInfo.TGT - motorInfo->pidInfo.ENC;
  motorInfo->pidInfo.PWM += motorInfo->pidInfo.Velocity_KP*(motorInfo->pidInfo.Bias-motorInfo->pidInfo.Last_bias)+motorInfo->pidInfo.Velocity_KI*motorInfo->pidInfo.Bias;
  motorInfo->pidInfo.Last_bias = motorInfo->pidInfo.Bias;
  if(motorInfo->pidInfo.TGT==0){
    motorInfo->pidInfo.PWM = 0;
    motorInfo->pidInfo.Bias = 0;
    motorInfo->pidInfo.Last_bias = 0;
  }
}

void range_PWM(motorInfoType *motorInfo, int16_t amplitude){
  if(motorInfo->pidInfo.PWM > amplitude){
    motorInfo->pidInfo.PWM = amplitude;
  }
  if(motorInfo->pidInfo.PWM < -amplitude){
    motorInfo->pidInfo.PWM = -amplitude;
  }
}

void set_PWM(motorInfoType *motorInfo){
  if(motorInfo->pidInfo.PWM>0){
    __HAL_TIM_SET_COMPARE(&motorInfo->pwm_a_htimx, motorInfo->pwm_a_channel_x, motorInfo->pidInfo.PWM);
    __HAL_TIM_SET_COMPARE(&motorInfo->pwm_b_htimx, motorInfo->pwm_b_channel_x, 0);
  } else if(motorInfo->pidInfo.PWM<0){
    __HAL_TIM_SET_COMPARE(&motorInfo->pwm_a_htimx, motorInfo->pwm_a_channel_x, 0);
    __HAL_TIM_SET_COMPARE(&motorInfo->pwm_b_htimx, motorInfo->pwm_b_channel_x, -motorInfo->pidInfo.PWM);
  } else{
    __HAL_TIM_SET_COMPARE(&motorInfo->pwm_a_htimx, motorInfo->pwm_a_channel_x, 0);
    __HAL_TIM_SET_COMPARE(&motorInfo->pwm_b_htimx, motorInfo->pwm_b_channel_x, 0);
  }

}

void init_motorInfo(motorInfoType *motorInfo, TIM_HandleTypeDef enc_htimx, TIM_HandleTypeDef pwm_a_htimx, TIM_HandleTypeDef pwm_b_htimx, uint16_t pwm_a_channel_x, uint16_t pwm_b_channel_x){
  motorInfo->pidInfo.ENC = 0;
  motorInfo->pidInfo.ADD = 0;
  motorInfo->pidInfo.TGT = 0;
  motorInfo->pidInfo.PWM = 0;
  motorInfo->pidInfo.Velocity_KP = 120;
  motorInfo->pidInfo.Velocity_KI = 30;
  motorInfo->enc_htimx = enc_htimx;
  motorInfo->pwm_a_htimx = pwm_a_htimx;
  motorInfo->pwm_a_channel_x = pwm_a_channel_x;
  motorInfo->pwm_b_htimx = pwm_b_htimx;
  motorInfo->pwm_b_channel_x = pwm_b_channel_x;
}

void Kinematic_Analysis_4(motorInfoType *motorInfoList, carInfoType *carInfo){
  float wide = (float) 0.18652, lenth = (float) 0.1525;
  motorInfoList[0].pidInfo.TGT = (int16_t) (carInfo->x + carInfo->y + carInfo->z * (wide + lenth));
  motorInfoList[1].pidInfo.TGT = (int16_t) (carInfo->x - carInfo->y - carInfo->z * (wide + lenth));
  motorInfoList[2].pidInfo.TGT = (int16_t) (carInfo->x + carInfo->y - carInfo->z * (wide + lenth));
  motorInfoList[3].pidInfo.TGT = (int16_t) (carInfo->x - carInfo->y + carInfo->z * (wide + lenth));

  motorInfoList[1].pidInfo.TGT = -motorInfoList[1].pidInfo.TGT;
  motorInfoList[2].pidInfo.TGT = -motorInfoList[2].pidInfo.TGT;
}

void start_motor(motorInfoType *motorInfoList, uint8_t len){
  for(uint8_t i=0; i<len; i++){
    HAL_TIM_Encoder_Start(&motorInfoList[i].enc_htimx, TIM_CHANNEL_1|TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&motorInfoList[i].pwm_a_htimx,motorInfoList[i].pwm_a_channel_x);
    HAL_TIM_PWM_Start(&motorInfoList[i].pwm_b_htimx,motorInfoList[i].pwm_b_channel_x);
  }
}

void motor_run(motorInfoType *motorInfoList, uint8_t len){
  for(uint8_t i=0; i<len; i++){
    set_ENC(&motorInfoList[i]);
//    set_ADD(&motorInfoList[i]);
    incremental_PI(&motorInfoList[i]);
    range_PWM(&motorInfoList[i], 7200);
    set_PWM(&motorInfoList[i]);
  }
}
