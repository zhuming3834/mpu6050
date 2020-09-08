var i2c = require('i2c-bus');

// MPU6050 寄存器
var PWR_MGMT_1 = 0x6B,
	PWR_MGMT_2 = 0x6C,
	SMPLRT_DIV = 0x19;
var    CONFIG = 0x1A,
  GYRO_CONFIG = 0x1B,
 ACCEL_CONFIG = 0x1C,
    INT_ENABLE = 0x38;
// 加速度  16位数据  低8位是在高8位的基础上地址 +1
var ACCEL_XOUT_H = 0x3B,
	ACCEL_YOUT_H = 0x3D,
	ACCEL_ZOUT_H = 0x3F;
// 角速度  16位数据  低8位是在高8位的基础上地址 +1
var GYRO_XOUT_H = 0x43,
	GYRO_YOUT_H = 0x45,
    GYRO_ZOUT_H = 0x47;
// 当前温度寄存器
var TEMP_OUT = 0x41;
// 加速度计和陀螺仪输出值和实际值的换算比例  输出值/比例 = 实际值
var ACCEL_LSB_SENSITIVITY = 16384.0,   // 加速度计 16384LSB/g
     GYRO_LSB_SENSITIVITY = 16.4;      // 陀螺仪 16.4LSB%s

// ±2g   ±4g   ±8g   ±16g
//16384  8192  4096  2048           LSB/g

// ±250%s   ±500%s  ±1000%s   ±2000%s
// 131      65.5    32.8      16.4  LSB%s


function mpu6050( i2cbus, mpuaddress ) {
	if (!(this instanceof mpu6050)) {
		return new mpu6050(i2cbus, mpuaddress);
	}
	this.address = mpuaddress;
  this.bus = i2c.openSync(i2cbus);

  // 内部20MHz时钟源 开启温度检测  关闭休眠
  this.bus.writeByteSync(this.address, PWR_MGMT_1, 0);
  // 采样分分频率 
  // SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV) where INTERNAL_SAMPLE_RATE = 1kHz
  this.bus.writeByteSync(this.address, SMPLRT_DIV, 7); // 125Hz
  // 正常工作模式
  this.bus.writeByteSync(this.address, PWR_MGMT_2, 0);
  this.bus.writeByteSync(this.address, CONFIG, 0);
  // 陀螺仪设置  不自检  ±2000dps
  this.bus.writeByteSync(this.address, GYRO_CONFIG, 24);  // 24 -->  0x18
  // 加速度计设置 不自检 ±2g
  this.bus.writeByteSync(this.address, ACCEL_CONFIG, 0); 
  // 数据就绪中断
  this.bus.writeByteSync(this.address, INT_ENABLE, 1);
}

// i2c读取mpu6050原始数据
mpu6050.prototype.read_raw_data = function (addr) {
	var high = this.bus.readByteSync(this.address, addr);
	var low = this.bus.readByteSync(this.address, addr+1);
	var value = (high << 8) + low; // 高8位 左移8位 组装成16位的数据
	if (value > 32768) {
		value = value - 65536;
	}
	return value;
};

// 读取温度值
mpu6050.prototype.get_temp = function() {
  var value = this.read_raw_data(TEMP_OUT);
  var temp = 36.53 + value/340.0;  // 输出 ℃
	var mpu6050_temp = {
		temp:temp
	}
	return mpu6050_temp;
}

// 读取角速度值
mpu6050.prototype.get_gyro_xyz = function() {
	var x = this.read_raw_data(GYRO_XOUT_H);
	var y = this.read_raw_data(GYRO_YOUT_H);
	var z = this.read_raw_data(GYRO_ZOUT_H);
	var gyro_xyz = {
		x:x,
		y:y,
		z:z
	}
	return gyro_xyz;
}

// 读取加速度值
mpu6050.prototype.get_accel_xyz = function() {
	var x = this.read_raw_data(ACCEL_XOUT_H);
	var y = this.read_raw_data(ACCEL_YOUT_H);
	var z = this.read_raw_data(ACCEL_ZOUT_H);
	var accel_xyz = {
		x:x,
		y:y,
		z:z
	}
	return accel_xyz;
}

mpu6050.prototype.get_roll_pitch_yaw = function( gyro_xyz, accel_xyz ) {
	var Ax = accel_xyz.x/ACCEL_LSB_SENSITIVITY;
	var Ay = accel_xyz.y/ACCEL_LSB_SENSITIVITY;
    var Az = accel_xyz.z/ACCEL_LSB_SENSITIVITY;
  
	var Gx = gyro_xyz.x/GYRO_LSB_SENSITIVITY;
	var Gy = gyro_xyz.y/GYRO_LSB_SENSITIVITY;
	var Gz = gyro_xyz.z/GYRO_LSB_SENSITIVITY;
	var roll = Ax*-100;
    var pitch = Ay*-100;
    var yaw = Az*-100;

	var roll_pitch_yaw = {
		roll: roll,   // 横滚角 精度:0.1° 范围:-180.0°<---> +180.0°
        pitch: pitch, // 俯仰角 精度:0.1° 范围:-90.0° <---> +90.0°
        yaw: yaw      // 航向角 精度:0.1° 范围:-180.0°<---> +180.0°
	}
	return roll_pitch_yaw;
}

// 获取MPU6050数据
mpu6050.prototype.get_mpu6050Data = function( gyro_xyz, accel_xyz ) {

  var gyro_xyz = this.get_gyro_xyz();
  var accel_xyz = this.get_accel_xyz();
  var temp = this.get_temp();
  var roll_pitch_yaw = this.get_roll_pitch_yaw(gyro_xyz,accel_xyz);

	var mpu6050Data = {
        gyro_xyz: gyro_xyz,
        accel_xyz: accel_xyz,
        temp: temp,
        roll_pitch_yaw: roll_pitch_yaw,
	}
	return mpu6050Data;
}


module.exports = mpu6050;
