int displayResults(int measurement, int i, int val){
  float voltage = ((5.000/1024) * val);
  float potValue = (10000/128) * i;
  Serial.println("Test #" + String(measurement));
  Serial.println("     Potentiometer Bit Value: " + String(i));
  Serial.println("     Potentiometer (Ohms): " + String(potValue));
  Serial.println("     Output Bit Value: " + String(val));
  Serial.println("     Voltage: " + String(voltage) + "V\n");
}
int digitalPotWrite(int value){
  SPI.beginTransaction(SPISettings(250000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS, LOW);
  SPI.transfer(address);
  SPI.transfer(value);
  digitalWrite(CS, HIGH);
  SPI.endTransaction();
}

String createCSV(int s1, int s2, int s3, int tNum) {
  dataString += String(s1);
  dataString += ",";
  dataString += String(s2);
  dataString += ",";
  dataString += String(s3);
  dataString += ",";
  dataString += String(tNum);
  dataString += "\n";
  return dataString;
}

int targetCheck(int val, int target, int i){
  //check for distance val is from the target
   //if the analog value is greater than 50% of the target value, decrease 'i' by 50
    if (val > 1.50 * target){
      i = i - 50;
    }
    //if the analog value is greater than 40% of the target value, decrease 'i' by 40
    else if (val > 1.40 * target){
      i = i - 40;
    }
    //if the analog value is greater than 30% of the target value, decrease 'i' by 30
    else if (val > 1.30 * target){
      i = i - 30;
    }
    //if the analog value is greater than 20% of the target value, decrease 'i' by 20
    else if (val > 1.20 * target){
      i = i - 20;
    }
    //if the analog value is greater than 10% of the target value, decrease 'i' by 10
    else if (val > 1.10 * target){
      i = i - 10;
    }
    //if the analog value is greater than 20% of the target value, decrease 'i' by 5
    else if (val > 1.04 * target){
      i = i - 5;
    }
    //if the analog value is greater than 20% of the target value, decrease 'i' by 1
    else if (val > target){
      i = i - 1;
    }
 //--------------------------------------------------------------------------------------------------------------
    //if the analog value is greater than 20% of the target value, decrease 'i' by 50
    else if (val < (1- 0.50) * target){
      i = i + 50;
    }
    else if (val < (1- 0.04) * target){
      i = i + 5;
    }
    //if the analog value is greater than 40% of the target value, decrease 'i' by 50
    else if (val < (1- 0.10) * target){
      i = i + 10;
    }
    //if the analog value is greater than 30% of the target value, decrease 'i' by 50
    else if (val < (1- 0.20) * target){
      i = i + 20;
    }
    //if the analog value is greater than 20% of the target value, decrease 'i' by 50
    else if (val < (1- 0.30) * target){
      i = i + 30;
    }
    //if the analog value is greater than 10% of the target value, decrease 'i' by 50
    else if (val < (1- 0.40) * target){
      i = i + 40;
    }
    else if (val < target){
      i = i + 1;
    }
    return i;
}
