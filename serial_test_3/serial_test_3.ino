const byte BufferSize = 150;
char Buffer[BufferSize + 1];

void setup(){
  Serial.begin(19200);
  Serial.setTimeout(2000);
}

void loop(){

  if(Serial.available() > 0){

    while(Serial.available() > 0){
      // "readBytes" terminates if the determined length has been read, or it
      // times out. It fills "Buffer" with 1 to 90 bytes of data. To change the
      // timeout use: Serial.setTimeout() in setup(). Default timeout is 1000ms.
      Serial.readBytes(Buffer, BufferSize);
    }

    // Print out "your" 90 char buffer's contents.
    Serial.println(Buffer);
    Serial.println("--");
    //Serial.println("");

    // You can use Serial.peek() to check if more than 90 chars
    // were in the serial buffer and "your" buffer has truncated data.
    // This should never happen because you know what the max length of
    // the incoming data is and you have adequately sized your input buffer.
//    if(Serial.peek() != -1){
//      Serial.print("90 byte Buffer Overflowed. ");
//    }

    // Clear out "your" buffer's contents.
    memset(Buffer, 0, sizeof(Buffer));
  }
}
