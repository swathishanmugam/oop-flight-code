#include "DataLogging.hpp"

const int chipSelect = BUILTIN_SDCARD;
File DataFile;

// int called = 0;

// void call(){
//     called = 1;
// }

// int IsCalled(){
//     return called;
// }

void DataLogSetup(){
    //Serial.begin(9600);
    while (!Serial) {
         ; // wait for serial port to connect.
     }
    if (!SD.begin(chipSelect)) {
        Serial.println("SD Card initialization failed!");
        return;
    }
    Serial.println("SD Card initialization done.");
    DataFile = SD.open("Data.txt", FILE_WRITE);
    DataFile.println("w_x, w_y, w_z, mag_x, mag_y, mag_z, de_I_x, de_I_y, de_I_z, pt_I_x, pt_I_y, pt_I_z");
    DataFile.close();
}

// float w_x, float w_y, float w_z, float mag_x, float mag_y, float mag_z, float de_I_x, float de_I_y, float de_I_z, float pt_I_x, float pt_I_y, float pt_I_z
void DataLog(float Data[12], int size)
{
    DataFile = SD.open("Data.txt", FILE_WRITE);
    if (DataFile) {
        for(int i = 0 ; i< size;i++) {
            DataFile.print(Data[i]);
            DataFile.print(", ");
        }
        DataFile.println();

        DataFile.close();
        Serial.println("done.");
    } else {
        // if the file didn't open, print an error:
        Serial.println("error opening Data.txt");
    }
    DataFile.close();
}