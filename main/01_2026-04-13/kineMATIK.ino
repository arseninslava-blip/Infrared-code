/*
// ==================== ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ====================
float degToRad(float deg) {
  return deg / RADIAN;
}

float radToDeg(float rad) {
  return rad * RADIAN;
}

// ==================== КИНЕМАТИКА ====================
float dpsToRpm(float dps) {
  return dps * 60.0f / 360.0f;
}

void omniMoveDirection(int32_t wheelSpeed, float direction, float omega) {
  float directionRad = direction / RADIAN;
  float speedFactor = 0.7071f;
  float Vx = wheelSpeed * speedFactor * cos(directionRad);
  float Vy = wheelSpeed * speedFactor * sin(directionRad);
  omniMove(Vx, Vy, omega);
  
  Serial.println("========== OMNI MOVE DIRECTION ==========");
  Serial.print("  Wheel Speed: "); Serial.print(wheelSpeed); Serial.println(" dps");
  Serial.print("  Direction: "); Serial.print(direction); Serial.println(" deg");
  Serial.print("  Omega: "); Serial.print(omega); Serial.println(" dps");
  Serial.print("  Calculated: Vx="); Serial.print(Vx); Serial.print(" dps, Vy="); Serial.print(Vy); Serial.println(" dps");
  Serial.println("==========================================");
}

void omniMove(float Vx_dps, float Vy_dps, float omega_dps) {
  float omega_rad_s = omega_dps / RADIAN;
  float V_omega = (omega_rad_s * ROBOT_RADIUS) / WHEEL_RADIUS * RADIAN;
  
  int32_t motor_dps[4];
  motor_dps[0] = (int32_t)(Vx_dps - Vy_dps - V_omega);
  motor_dps[1] = (int32_t)(Vx_dps + Vy_dps + V_omega);
  motor_dps[2] = (int32_t)(Vx_dps - Vy_dps + V_omega);
  motor_dps[3] = (int32_t)(Vx_dps + Vy_dps - V_omega);
  
  for (int i = 0; i < 4; i++) {
    motor_dps[i] = constrain(motor_dps[i], -2000, 2000);
    setMotorSpeed(i, motor_dps[i]);
    delay(5);  // Небольшая задержка между отправками команд моторам
  }
  
  Serial.println("========== OMNI MOVE ==========");
  Serial.print("  Vx="); Serial.print(Vx_dps); Serial.print(" dps");
  Serial.print("  Vy="); Serial.print(Vy_dps); Serial.print(" dps");
  Serial.print("  ω="); Serial.print(omega_dps); Serial.println(" dps");
  Serial.print("  Motor dps: M1="); Serial.print(motor_dps[0]);
  Serial.print(" M2="); Serial.print(motor_dps[1]);
  Serial.print(" M3="); Serial.print(motor_dps[2]);
  Serial.print(" M4="); Serial.println(motor_dps[3]);
  Serial.println("================================");
  
  buffer_clear();
  drawRect(0, 0, 128, 64, true, false);
  drawString(5, 5, "OMNI PLATFORM", true);
  
  char buf[20];
  sprintf(buf, "Vx:%4.0f", Vx_dps); drawString(5, 16, buf, true);
  sprintf(buf, "Vy:%4.0f", Vy_dps); drawString(5, 26, buf, true);
  sprintf(buf, "W:%4.0f", omega_dps); drawString(5, 36, buf, true);
  
  sprintf(buf, "M1:%4d", motor_dps[0]); drawString(70, 16, buf, true);
  sprintf(buf, "M2:%4d", motor_dps[1]); drawString(70, 26, buf, true);
  sprintf(buf, "M3:%4d", motor_dps[2]); drawString(70, 36, buf, true);
  sprintf(buf, "M4:%4d", motor_dps[3]); drawString(70, 46, buf, true);
  
  display();
}


void rotate(float omega_dps) {

  for (int i = 0; i < 4; i++) {
    if(i > 1){
      setMotorSpeed(i, omega_dps);
    } else {
      setMotorSpeed(i, -omega_dps);
    }
    delay(5);  // Небольшая задержка между отправками команд моторам
  }
}
void omniStop() {
  allMotorsStop();
  
  Serial.println("[OMNI] STOP");
  
  buffer_clear();
  drawRect(0, 0, 128, 64, true, false);
  drawString(50, 28, "STOP", true);
  display();
}
*/