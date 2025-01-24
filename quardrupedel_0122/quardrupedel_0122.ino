#include <Wire.h>
#include <EEPROM.h>
#include "ServoEasing.hpp"

#define servo_driver_bits 4096

const int servo_pins[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

ServoEasing servos[] = {
  ServoEasing(0x40, &Wire), ServoEasing(0x40, &Wire), ServoEasing(0x40, &Wire),
  ServoEasing(0x40, &Wire), ServoEasing(0x40, &Wire), ServoEasing(0x40, &Wire),
  ServoEasing(0x40, &Wire), ServoEasing(0x40, &Wire), ServoEasing(0x40, &Wire),
  ServoEasing(0x40, &Wire), ServoEasing(0x40, &Wire), ServoEasing(0x40, &Wire)
};

int servo_cal_val[][2] = {
  {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
  {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}
};

const int rows = sizeof(servo_cal_val) / sizeof(servo_cal_val[0]); // 데이터 행의 개수
const int cols = 2; // 열의 개수
int eeprom_val_tmp[3];

float servo_angles[] = {90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90};
int servo_moving_time = 500;
int delay_time = 15;
bool stop_flag = false;

String inString;

void setup() {
  Serial.begin(115200);
  servo_setup();
  delay(100);
}

void loop() {
  delay(100);
  if (Serial.available() > 0) {
    inString = Serial.readStringUntil('\n');
    char cmd = inString[0];
    if (cmd == '1') {
      // 각 서보 각도 설정
      for (int i = 0; i < 12; i++) {
        char startChar = 'a' + i;
        char endChar = 'a' + i + 1;
        servo_angles[i] = inString.substring(inString.indexOf(startChar) + 1, inString.indexOf(endChar)).toFloat();
        servo_angles[i] = constrain(servo_angles[i], 0, 180);
      }
      stop_flag = false;
      while (!stop_flag) {
        bool all_done = true;
        for (int i = 0; i < 12; i++) {
          all_done &= servos[i].startEaseToD(servo_angles[i], servo_moving_time);
        }
        if (all_done) break;
        delay(delay_time);
      }
    }else if (cmd == '8') {
      Serial.println("");
      Serial.println("cmd 8");
      update_calib_val();  // EEPROM 읽기 및 업데이트

    } else if (cmd == '9') {
      for (int i = 0; i < 3; i++) {
        char startChar = 'a' + i;
        char endChar = 'a' + i + 1;
        eeprom_val_tmp[i] = inString.substring(inString.indexOf(startChar) + 1, inString.indexOf(endChar)).toInt();
      }
      update_eeprom_val(eeprom_val_tmp[0], eeprom_val_tmp[1], eeprom_val_tmp[2]);
      delay(1000);
      update_calib_val();
    }
  }
}

void servo_setup(){
  update_calib_val();
  delay(500);
  for (int i = 0; i < 12; i++) {
    servos[i].attach(servo_pins[i],servo_cal_val[i][0],servo_cal_val[i][1],180,0);
    servos[i].setEasingType(EASE_CUBIC_IN_OUT);
    servos[i].write(servo_angles[i]);
  }
};

void update_calib_val() {
  for (int i = 0; i < 12; i++) {
    // 4바이트씩 읽어서 두 값을 추출
    int val1 = (EEPROM.read(i * 4) + (EEPROM.read(i * 4 + 1) << 8)); // 첫 번째 값
    int val2 = (EEPROM.read(i * 4 + 2) + (EEPROM.read(i * 4 + 3) << 8)); // 두 번째 값

    servo_cal_val[i][0]=val1;
    servo_cal_val[i][1]=val2;

    Serial.print("Index ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(val1);
    Serial.print(", ");
    Serial.println(val2);
  }
  Serial.println("");
}

void save_eeprom(int arr[][2], int row_count, int col_count) {
  int eepromIndex = 0;
  for (int i = 0; i < row_count; i++) {
    for (int j = 0; j < col_count; j++) {
      EEPROM.write(eepromIndex, lowByte(arr[i][j]));  // 첫 번째 바이트
      EEPROM.write(eepromIndex + 1, highByte(arr[i][j])); // 두 번째 바이트
      eepromIndex += 2; // 인덱스 2씩 증가
    }
  }
}

void update_eeprom_val(int index, int value1, int value2) {
  int eepromIndex = index * 4;  // 한 인덱스는 4바이트, 인덱스 * 4
  
  EEPROM.write(eepromIndex, lowByte(value1));    // 첫 번째 값 (하위 바이트)
  EEPROM.write(eepromIndex + 1, highByte(value1)); // 첫 번째 값 (상위 바이트)
  EEPROM.write(eepromIndex + 2, lowByte(value2));    // 두 번째 값 (하위 바이트)
  EEPROM.write(eepromIndex + 3, highByte(value2)); // 두 번째 값 (상위 바이트)
}


