#define NBR_COLLECTED_SENSORS 3

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




