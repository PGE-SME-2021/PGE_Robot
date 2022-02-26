#include <IMU/i2c.h>
#include <ros/ros.h>

i2c::i2c(int adapter_nr, int addr) {
    file_num = adapter_nr;
    this->addr = addr;
}


int i2c::read_8bit_word(int reg) {
    int file;
    char filename[20];
    //------ Open I2C BUS ------

    snprintf(filename, 19, "/dev/i2c-%d", file_num);

    file = open(filename, O_RDWR);

    if (file < 0) {
        ROS_ERROR("Failed to open the i2c bus.\n");
        close(file);
        return -1;
    }

    if (ioctl(file, I2C_SLAVE, addr) < 0) {
        ROS_ERROR("Failed to acquire bus access.\n");
        
        close(file);
        return -1;
    }

   
    int result_volt = i2c_smbus_read_byte_data(file, reg);
    if (result_volt < 0) {
        ROS_ERROR("Failed to read from register: 0x%02X", reg);
        close(file);
        return -1;
    }
    close(file);
    return result_volt;
}


int i2c::write_8bit_word(int reg, int data) {
    int file;
    char filename[20];
    //------ Open I2C BUS ------

    snprintf(filename, 19, "/dev/i2c-%d", file_num);

    file = open(filename, O_RDWR);

    if (file < 0) {
        ROS_ERROR("Failed to open the i2c bus.\n");
        close(file);
        return -1;
    }

    if (ioctl(file, I2C_SLAVE, addr) < 0) {
        ROS_ERROR("Failed to acquire bus access.\n");
        
        close(file);
        return -1;
    }

    //------ READ BYTES with SMBus ----
    int result_volt = i2c_smbus_write_byte_data(file, reg, data);
    if (result_volt < 0) {
        ROS_ERROR("Failed to write from register: 0x%02X", reg);
        close(file);
        return -1;
    }
    close(file);
    return result_volt;
}



float i2c::getAres(uint8_t Ascale) {
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    case AFS_2G:
      _aRes = 2.0f / 32768.0f;
      return _aRes;
      break;
    case AFS_4G:
      _aRes = 4.0f / 32768.0f;
      return _aRes;
      break;
    case AFS_8G:
      _aRes = 8.0f / 32768.0f;
      return _aRes;
      break;
    case AFS_16G:
      _aRes = 16.0f / 32768.0f;
      return _aRes;
      break;
  }
}

float i2c::getGres(uint8_t Gscale) {
  switch (Gscale)
  {
    case GFS_15_125DPS:
      _gRes = 15.125f / 32768.0f;
      return _gRes;
      break;
    case GFS_31_25DPS:
      _gRes = 31.25f / 32768.0f;
      return _gRes;
      break;
    case GFS_62_5DPS:
      _gRes = 62.5f / 32768.0f;
      return _gRes;
      break;
    case GFS_125DPS:
      _gRes = 125.0f / 32768.0f;
      return _gRes;
      break;
    case GFS_250DPS:
      _gRes = 250.0f / 32768.0f;
      return _gRes;
      break;
    case GFS_500DPS:
      _gRes = 500.0f / 32768.0f;
      return _gRes;
      break;
    case GFS_1000DPS:
      _gRes = 1000.0f / 32768.0f;
      return _gRes;
      break;
    case GFS_2000DPS:
      _gRes = 2000.0f / 32768.0f;
      return _gRes;
      break;
  }
}


void i2c::reset()
{
  // reset device
  int temp = read_8bit_word(ICM42605_DEVICE_CONFIG);
      temp = write_8bit_word(ICM42605_DEVICE_CONFIG, temp | 0x01);
  
}


uint8_t i2c::status()
{
  // reset device
  int temp = read_8bit_word(ICM42605_INT_STATUS);
  return temp;
}


void i2c::init(uint8_t Ascale, uint8_t Gscale, uint8_t AODR, uint8_t GODR)
{

   getAres(Ascale);
   getGres(Gscale);
   int temp = read_8bit_word(ICM42605_PWR_MGMT0);
   temp = write_8bit_word(ICM42605_PWR_MGMT0, temp | 0x0F); 



   temp = read_8bit_word(ICM42605_GYRO_CONFIG0);
   temp = write_8bit_word(ICM42605_GYRO_CONFIG0, GODR | Gscale << 5);  


   temp = read_8bit_word(ICM42605_ACCEL_CONFIG0);
   temp = write_8bit_word(ICM42605_ACCEL_CONFIG0, temp | AODR | Ascale << 5);  


   temp = read_8bit_word(ICM42605_GYRO_CONFIG1);
   temp = write_8bit_word(ICM42605_GYRO_CONFIG1, temp | 0xD0);  

 
   temp = read_8bit_word(ICM42605_INT_CONFIG);
   temp = write_8bit_word(ICM42605_INT_CONFIG, temp | 0x18 | 0x03);  

  
   temp = read_8bit_word(ICM42605_INT_CONFIG1);
   temp = write_8bit_word(ICM42605_INT_CONFIG1, temp & ~(0x10));  

  
   temp = read_8bit_word(ICM42605_INT_SOURCE0);
   temp = write_8bit_word(ICM42605_INT_SOURCE0, temp | 0x08);  

   
   temp = read_8bit_word(ICM42605_INT_SOURCE3);
   temp = write_8bit_word(ICM42605_INT_SOURCE3, temp | 0x01);  

  
   temp = read_8bit_word(ICM42605_REG_BANK_SEL);
   temp = write_8bit_word(ICM42605_REG_BANK_SEL, temp | 0x04);  

  // Select Bank 4
  
   temp = read_8bit_word(ICM42605_APEX_CONFIG5);
   temp = write_8bit_word(ICM42605_APEX_CONFIG5, temp & ~(0x07)); 

  

   temp = read_8bit_word(ICM42605_REG_BANK_SEL);
   temp = write_8bit_word(ICM42605_REG_BANK_SEL, temp & ~(0x07));

   temp = read_8bit_word(ICM42605_ACCEL_CONFIG_STATIC2);
   temp = write_8bit_word(ICM42605_ACCEL_CONFIG_STATIC2, temp |(0x01));

   temp = read_8bit_word(ICM42605_ACCEL_CONFIG_STATIC2);
   temp = write_8bit_word(ICM42605_ACCEL_CONFIG_STATIC2, temp |(0x24));

}


void i2c::offsetBias(float * dest1, float * dest2)
{
  int16_t temp[6] = {0, 0, 0, 0, 0, 0};
  int32_t sum[6] = {0, 0, 0, 0, 0, 0};


  ros::Duration(4).sleep();

  for (int ii = 0; ii < 128; ii++)
  {
    readData(temp);
    sum[0] += temp[0];
    sum[1] += temp[1];
    sum[2] += temp[2];
    sum[3] += temp[3];
    sum[4] += temp[4];
    sum[5] += temp[5];

    ros::Duration(0.05).sleep();
  }

  dest1[0] = sum[1] * _aRes / 128.0f;
  dest1[1] = sum[2] * _aRes / 128.0f;
  dest1[2] = sum[3] * _aRes / 128.0f;
  dest2[0] = sum[4] * _gRes / 128.0f;
  dest2[1] = sum[5] * _gRes / 128.0f;
  dest2[2] = sum[6] * _gRes / 128.0f;

  if (dest1[0] > 0.8f)  {
    dest1[0] -= 1.0f; // Remove gravity from the x-axis accelerometer bias calculation
  }
  if (dest1[0] < -0.8f) {
    dest1[0] += 1.0f; // Remove gravity from the x-axis accelerometer bias calculation
  }
  if (dest1[1] > 0.8f)  {
    dest1[1] -= 1.0f; // Remove gravity from the y-axis accelerometer bias calculation
  }
  if (dest1[1] < -0.8f) {
    dest1[1] += 1.0f; // Remove gravity from the y-axis accelerometer bias calculation
  }
  if (dest1[2] > 0.8f)  {
    dest1[2] -= 1.0f; // Remove gravity from the z-axis accelerometer bias calculation
  }
  if (dest1[2] < -0.8f) {
    dest1[2] += 1.0f; // Remove gravity from the z-axis accelerometer bias calculation
  }

}


void i2c::readData(int16_t * destination)
{
  uint8_t rawData[12];  // x/y/z accel register data stored here
  
  for (int i = 0; i < 12; i++) 
{ 
  rawData[i] =  read_8bit_word(0x1F + i );  // Read the 14 raw data registers into data array
  
}
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
  destination[3] = ((int16_t)rawData[6] << 8) | rawData[7] ;
  destination[4] = ((int16_t)rawData[8] << 8) | rawData[9] ;
  destination[5] = ((int16_t)rawData[10] << 8) | rawData[11] ;
  


   ;
}

