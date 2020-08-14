  // ------------------------------
  // ----------- Limits -----------
  // ------------------------------

  if(!digitalRead(pinSwich)){
    Serial.println("pinSwitch ON");
    pinSwichOn = 1;
    tone(buzzer, 1950); 
  }else{
    pinSwichOn = 0;
    noTone(buzzer); 
  }

  baseStepMove1 = 0;
  baseStepMove2 = 0;

    if((ps2x.Analog(PSS_RX) == 255)){  
  
      if(pinSwichOn){
       baseStepMove1 = 0;     

      }else{
        baseStepMove1 = 1;     
      }
      
      if(!pinSwichOn || !baseStepMove2){
        basePos = basePos+1;
      }
    }
  

  if((ps2x.Analog(PSS_RX) == 0)){

    
      if(pinSwichOn){
       baseStepMove2 = 0;     
      }else{
       baseStepMove2 = 1;     
      }

    if(!pinSwichOn || !baseStepMove1){
      basePos = basePos-1;      
    }

  }