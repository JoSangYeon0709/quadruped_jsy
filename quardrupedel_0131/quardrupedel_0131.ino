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

float servo_angles[] = {90.0, 86.3, 56.0, 90.0, 93.7, 124.0, 90.0, 94.5, 64.2, 90.0, 85.5, 115.8};
int servo_moving_time = 1;
int delay_time = 15;
bool stop_flag = false;

String inString;
String inStrings[12];  // 여러 데이터를 담을 배열

void setup() {
  Serial.begin(115200);
  servo_setup();
  delay(100);
}

void loop() {
  if (Serial.available() > 0) {
    inString = Serial.readStringUntil('\n');  // 한 줄의 입력을 읽음
    char cmd = inString[0];
    
    if (cmd == '1') {
      // 각 서보 각도 설정
      for (int i = 0; i < 12; i++) {
        char startChar = 'a' + i;  // 'a'부터 시작하여 각 서보의 각도를 추출
        char endChar = 'a' + i + 1;
        int startPos = inString.indexOf(startChar) + 1;
        int endPos = inString.indexOf(endChar);

        // 데이터 추출 및 배열에 저장
        if (startPos != -1 && endPos != -1 && endPos > startPos) {
          inStrings[i] = inString.substring(startPos, endPos);  // 데이터 부분만 추출하여 배열에 저장
        } else {
          inStrings[i] = "0";  // 잘못된 형식일 경우 기본값 0 저장
        }
      }

      // 서보 각도 적용
      for (int i = 0; i < 12; i++) {
        servo_angles[i] = inStrings[i].toFloat();  // String을 float으로 변환하여 각도 적용
        servo_angles[i] = constrain(servo_angles[i], 0, 180);  // 0~180으로 제한
        servos[i].startEaseToD(servo_angles[i], servo_moving_time);  // 서보 이동
      }
    }
    else if (cmd == '8') {
      Serial.println("");
      Serial.println("cmd 8");
      update_calib_val();  // EEPROM 읽기 및 업데이트
    } 
    else if (cmd == '9') {
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
    servos[i].attach(servo_pins[i], servo_cal_val[i][0], servo_cal_val[i][1], 180, 0);
    servos[i].setEasingType(EASE_CUBIC_IN_OUT);
    servos[i].write(servo_angles[i]);
  }
}

void update_calib_val() {
  for (int i = 0; i < 12; i++) {
    // 4바이트씩 읽어서 두 값을 추출
    int val1 = (EEPROM.read(i * 4) + (EEPROM.read(i * 4 + 1) << 8)); // 첫 번째 값
    int val2 = (EEPROM.read(i * 4 + 2) + (EEPROM.read(i * 4 + 3) << 8)); // 두 번째 값

    servo_cal_val[i][0] = val1;
    servo_cal_val[i][1] = val2;

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
