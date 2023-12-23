#include <ezButton.h>

ezButton redBtn(32);
ezButton swBtn(33);

void setup(){
  Serial.begin(9600);
  redBtn.setDebounceTime(50);
  swBtn.setDebounceTime(50);
}

void loop(){
  redBtn.loop();
  swBtn.loop();

    int redBtnState = redBtn.getState();
    int swBtnState = swBtn.getState();

    Serial.print("RedButton State: ");
    Serial.println(redBtnState);
    Serial.print("SwButton State: ");
    Serial.println(swBtnState);
}
