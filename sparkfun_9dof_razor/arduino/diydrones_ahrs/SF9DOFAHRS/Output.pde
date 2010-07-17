void printdata(void)
{    
      Serial.print("!");

      #if PRINT_IMU == 1
      Serial.print("IMU:");
      Serial.print(Magneto_Vector[0]);
      Serial.print(",");
      Serial.print(Magneto_Vector[1]);
      Serial.print(",");
      Serial.print(Magneto_Vector[2]);
      Serial.print(",");
      Serial.print(Gyro_Vector[0]);
      Serial.print(",");
      Serial.print(Gyro_Vector[1]);
      Serial.print(",");
      Serial.print(Gyro_Vector[2]);
      Serial.print(",");
      Serial.print(Accel_Vector[0]);
      Serial.print(",");
      Serial.print(Accel_Vector[1]);
      Serial.print(",");
      Serial.print(Accel_Vector[2]);
      #endif      
      #if PRINT_ANALOGS==1
      Serial.print(",AN:");
      Serial.print(AN[sensors[0]]);  //(int)read_adc(0)
      Serial.print(",");
      Serial.print(AN[sensors[1]]);
      Serial.print(",");
      Serial.print(AN[sensors[2]]);  
      Serial.print(",");
      Serial.print(ACC[0]);
      Serial.print (",");
      Serial.print(ACC[1]);
      Serial.print (",");
      Serial.print(ACC[2]);
      Serial.print(",");
      Serial.print(magnetom_x);
      Serial.print (",");
      Serial.print(magnetom_y);
      Serial.print (",");
      Serial.print(magnetom_z);      
      #endif
      Serial.println();    
      
}

long convert_to_dec(float x)
{
  return x*10000000;
}

