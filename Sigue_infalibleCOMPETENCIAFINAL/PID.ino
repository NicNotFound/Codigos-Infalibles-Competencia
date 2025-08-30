

void PID(uint16_t position) {
  int error = position - 3500;
  integral += error;
  int derivative = error - lastError;
  int output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  lastError = error;

  int vIzq = constrain(velocidadBase + output, 0, 255);
  int vDer = constrain(velocidadBase - output, 0, 255);
  Motor(vIzq, vDer);
}