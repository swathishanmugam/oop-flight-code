#include "DataLogging.hpp"

const int chipSelect = BUILTIN_SDCARD;
File DataFile;

void DataLogSetup(){
    Serial.begin(9600);
    while (!Serial) {
        ; // wait for serial port to connect.
    }
    if (!SD.begin(chipSelect)) {
#ifdef VERBOSE
        Serial.println("SD Card initialization failed!");
#endif
        return;
    }
#ifdef VERBOSE
    Serial.println("SD Card initialization done.");
#endif
    DataFile = SD.open("Data.txt", FILE_WRITE);
    DataFile.println("w_x, w_y, w_z, mag_x, mag_y, mag_z");
    DataFile.close();
}

void DataLog(){
    DataFile = SD.open("Data.txt", FILE_WRITE);
    if (DataFile) {
        DataFile.print(sfr::imu::gyro_x * 3.14159 / 180.0);
        DataFile.print(", ");
        DataFile.print(sfr::imu::gyro_y * 3.14159 / 180.0);
        DataFile.print(", ");
        DataFile.print(sfr::imu::gyro_z * 3.14159 / 180.0);
        DataFile.print(", ");
        DataFile.print(sfr::imu::mag_x);
        DataFile.print(", ");
        DataFile.print(sfr::imu::mag_y);
        DataFile.print(", ");
        DataFile.println(sfr::imu::mag_z);

        DataFile.close();
#ifdef VERBOSE
        Serial.println("done.");
#endif
    } else {
        // if the file didn't open, print an error:
#ifdef VERBOSE
        Serial.println("error opening Data.txt");
#endif
    }
    DataFile.close();
}