#define NBR_COLLECTED_SENSORS 3
// {"Z":"Dock","S":"P_bmp180","V":997.00,"R":""}

struct sensor_entry {
    String name;
    String zone;
    String sensor;
    float value;  
};

typedef struct sensor_entry sensor_info;

sensor_info collect_sens[NBR_COLLECTED_SENSORS] = {
    {"Indoor Temp","Tupa","Temp",24.2},
    {"Outdoor Temp","OD_1","Temp",22.1},
    {"Water Temp","Dock","Temp",16.0}
};

void test_sens_db(void){
    Serial.println("Test sens_db");
    for( int i = 0; i < NBR_COLLECTED_SENSORS; i++){          
        Serial.print(collect_sens[i].name);
        Serial.print(" - ");
        Serial.print(collect_sens[i].zone);
        Serial.print(" - ");
        Serial.print(collect_sens[i].sensor);
        Serial.print(" - ");
        Serial.print(collect_sens[i].value);
        Serial.println();
    }
}
#define MSG_ATTRIBUTES 4
void parse_msg(char *rad_msg){
   int attr_pos[MSG_ATTRIBUTES];
   int attr_end[MSG_ATTRIBUTES];
   String attributes[MSG_ATTRIBUTES];
   String msg_str;

   
   msg_str = rad_msg;
   Serial.print("String= "); Serial.println(msg_str);
   attr_pos[0] = msg_str.indexOf("{\"Z\":") + 5;
   attr_pos[1] = msg_str.indexOf(",\"S\":") + 5;
   attr_pos[2] = msg_str.indexOf(",\"V\":") + 5;
   attr_pos[3] = msg_str.indexOf(",\"R\":") + 5;
   boolean attr_found = true;
   Serial.println("Search Attributes");
   for(int i = 0; i<MSG_ATTRIBUTES;i++){
      if(attr_pos[i] < 0){
        attr_found = false;
        Serial.println("Attribute[i] not found");
      } 
   }
   if (attr_found){
     for(int i = 0; i < MSG_ATTRIBUTES;i++){
       int end_pos = -1;
       Serial.println(i);
       if (msg_str.charAt(attr_pos[i]) == '\"'){
         attr_pos[i]++;
         end_pos = msg_str.indexOf('\"',attr_pos[i]);
       }  
       else {  //not a string value
         end_pos = msg_str.indexOf(',',attr_pos[i]);
         if end_pos < 0 end_pos = msg_str.indexOf('}',attr_pos[i]);
       }
       if (end_pos < attr_pos[i]){
         attr_found = false;
         Serial.println("missing double qoute");
       } else {
         attributes[i] = msg_str.substring(attr_pos[i],end_pos);
         Serial.print(i); Serial.print(" -- ");
         Serial.println(attributes[i]);
       }
     }
   }
   else {
     Serial.println("Missing JSON tags");
   }   
   
   //Serial.println(msg_str.indexOf("{\"Z\":"));
   //Serial.println(msg_str.indexOf("\"S\":"));
   //Serial.println(msg_str.indexOf("\"V\":"));
   //Serial.println(msg_str.indexOf("\"R\":"));
   //Serial.println(msg_str.indexOf("Dock"));
   //Serial.println(msg_str.indexOf("OD_1"));
   
   //NBR_COLLECTED_SENSORS
}
