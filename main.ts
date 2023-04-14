/*
ardubits extended library for microbit
*/

enum RGB {
    //% block="红"
    RED,
    //% block="绿"
    GREEN,
    //% block="蓝"
    BLUE,
    //% block="全部"
    CLEAR
}

enum AXIS {
    X = 1,
    Y = 2,
    Z = 3
}


/*
 * extended block
 */
//% color="#13c2c2" weight=10 icon="\uf0e7"
//% groups='["IIC Communication", "Basic Sensor"]'
namespace DLM {


    enum LCS_Constants {
        // Constants
        ADDRESS = 0x29,
        ID = 0x12, // Register should be equal to 0x44 for the TCS34721 or TCS34725, or 0x4D for the TCS34723 or TCS34727.

        COMMAND_BIT = 0x80,

        ENABLE = 0x00,
        ENABLE_AIEN = 0x10, // RGBC Interrupt Enable
        ENABLE_WEN = 0x08, // Wait enable - Writing 1 activates the wait timer
        ENABLE_AEN = 0x02, // RGBC Enable - Writing 1 actives the ADC, 0 disables it
        ENABLE_PON = 0x01, // Power on - Writing 1 activates the internal oscillator, 0 disables it
        ATIME = 0x01, // Integration time
        WTIME = 0x03, // Wait time (if ENABLE_WEN is asserted)
        AILTL = 0x04, // Clear channel lower interrupt threshold
        AILTH = 0x05,
        AIHTL = 0x06, // Clear channel upper interrupt threshold
        AIHTH = 0x07,
        PERS = 0x0C, // Persistence register - basic SW filtering mechanism for interrupts
        PERS_NONE = 0x00, // Every RGBC cycle generates an interrupt
        PERS_1_CYCLE = 0x01, // 1 clean channel value outside threshold range generates an interrupt
        PERS_2_CYCLE = 0x02, // 2 clean channel values outside threshold range generates an interrupt
        PERS_3_CYCLE = 0x03, // 3 clean channel values outside threshold range generates an interrupt
        PERS_5_CYCLE = 0x04, // 5 clean channel values outside threshold range generates an interrupt
        PERS_10_CYCLE = 0x05, // 10 clean channel values outside threshold range generates an interrupt
        PERS_15_CYCLE = 0x06, // 15 clean channel values outside threshold range generates an interrupt
        PERS_20_CYCLE = 0x07, // 20 clean channel values outside threshold range generates an interrupt
        PERS_25_CYCLE = 0x08, // 25 clean channel values outside threshold range generates an interrupt
        PERS_30_CYCLE = 0x09, // 30 clean channel values outside threshold range generates an interrupt
        PERS_35_CYCLE = 0x0A, // 35 clean channel values outside threshold range generates an interrupt
        PERS_40_CYCLE = 0x0B, // 40 clean channel values outside threshold range generates an interrupt
        PERS_45_CYCLE = 0x0C, // 45 clean channel values outside threshold range generates an interrupt
        PERS_50_CYCLE = 0x0D, // 50 clean channel values outside threshold range generates an interrupt
        PERS_55_CYCLE = 0x0E, // 55 clean channel values outside threshold range generates an interrupt
        PERS_60_CYCLE = 0x0F, // 60 clean channel values outside threshold range generates an interrupt
        CONFIG = 0x0D,
        CONFIG_WLONG = 0x02, // Choose between short and long (12x) wait times via WTIME
        CONTROL = 0x0F, // Set the gain level for the sensor
        STATUS = 0x13,
        STATUS_AINT = 0x10, // RGBC Clean channel interrupt
        STATUS_AVALID = 0x01, // Indicates that the RGBC channels have completed an integration cycle

        CDATAL = 0x14, // Clear channel data
        CDATAH = 0x15,
        RDATAL = 0x16, // Red channel data
        RDATAH = 0x17,
        GDATAL = 0x18, // Green channel data
        GDATAH = 0x19,
        BDATAL = 0x1A, // Blue channel data
        BDATAH = 0x1B,

        GAIN_1X = 0x00, //  1x gain
        GAIN_4X = 0x01, //  4x gain
        GAIN_16X = 0x02, // 16x gain
        GAIN_60X = 0x03  // 60x gain
    }

    export enum REGISTER {
        // 陀螺仪采样率地址
        SMPLRT_DIV = 0x19,
        // 低通滤波频率地址
        CONFIG = 0x1a,
        // 陀螺仪自检及测量范围
        GYRO_CONFIG = 0x1b,
        // 加速计自检、测量范围及高通滤波频率
        ACCEL_CONFIG = 0x1c,
        // 电源管理地
        PWR_MGMT = 0x6b,
        ACCEL_X = 0x3b,
        ACCEL_Y = 0x3d,
        ACCEL_Z = 0x3f,
        TEMPATURE = 0x41,
        GYRO_X = 0x43,
        GYRO_Y = 0x45,
        GYRO_Z = 0x47
    }





    let SMPLRT = 0x07 // 陀螺仪采样率 125Hz
    let CONFIG = 0x06 // 低通滤波频率 5Hz
    let GYRO_CONFIG = 0x18 // 典型值：0x18(不自检，2000deg/s) 
    let ACCEL_CONFIG = 0x01 // 典型值：0x01(不自检，2G，5Hz)
    let X_ACCEL_OFFSET = 0
    let Y_ACCEL_OFFSET = 0
    let Z_ACCEL_OFFSET = 0
    let X_GYRO_OFFSET = 0
    let Y_GYRO_OFFSET = 0
    let Z_GYRO_OFFSET = 0
    let MPU6050_I2C_ADDRESS = 0x68
    let LCS_integration_time_val = 0

    // I2C functions

    function I2C_WriteReg8(addr: number, reg: number, val: number) {
        let buf = pins.createBuffer(2)
        buf.setNumber(NumberFormat.UInt8BE, 0, reg)
        buf.setNumber(NumberFormat.UInt8BE, 1, val)
        pins.i2cWriteBuffer(addr, buf)
    }

    function I2C_ReadReg8(addr: number, reg: number): number {
        let buf = pins.createBuffer(1)
        buf.setNumber(NumberFormat.UInt8BE, 0, reg)
        pins.i2cWriteBuffer(addr, buf)
        buf = pins.i2cReadBuffer(addr, 1)
        return buf.getNumber(NumberFormat.UInt8BE, 0);
    }

    function I2C_ReadReg16(addr: number, reg: number): number {
        let buf = pins.createBuffer(1)
        buf.setNumber(NumberFormat.UInt8BE, 0, reg)
        pins.i2cWriteBuffer(addr, buf)
        buf = pins.i2cReadBuffer(addr, 2)
        // Little endian
        return ((buf.getNumber(NumberFormat.UInt8BE, 1) << 8) | buf.getNumber(NumberFormat.UInt8BE, 0));
    }



    function i2cWrite(addr: number, reg: number, value: number): void {
        let buf = pins.createBuffer(2);
        buf[0] = reg;
        buf[1] = value;
        pins.i2cWriteBuffer(addr, buf);
    }

    function i2cRead(addr: number, reg: number): number {
        pins.i2cWriteNumber(addr, reg, NumberFormat.UInt8BE);
        let val = pins.i2cReadNumber(addr, NumberFormat.UInt8BE);
        return val;
    }

    function checkAddress(addr: MPU6050_I2C_ADDRESS): boolean {
        switch (addr) {
            case MPU6050_I2C_ADDRESS.ADDR_0x68:
                return true
            case MPU6050_I2C_ADDRESS.ADDR_0x69:
                return true
            default:
                return false
        }
        return false
    }

    /**
	 * 初始化MPU6050
	 * @param addr [0-1] choose address; eg: MPU6050.MPU6050_I2C_ADDRESS.ADDR_0x68
	*/
    //% blockId="MPU6050_initMPU6050"
    //% block="初始化MPU6050"
    //% group="IIC Communication" weight=85
    export function initMPU6050(addr: MPU6050_I2C_ADDRESS) {
        if (checkAddress(addr)) {
            i2cWrite(addr, REGISTER.PWR_MGMT, 0)
        }
    }



    /**
	 *Read byte from MPU6050 register
	 * @param reg  register of MPU6050; eg: 0, 15, 23
	*/
    function readByte(addr: MPU6050_I2C_ADDRESS, reg: REGISTER): number {
        let val2 = i2cRead(addr, reg);
        return val2;
    }

    /**
	 *Read data from MPU6050 register
	 * @param reg  register of MPU6050; eg: 0, 15, 23
	*/
    function readWord(addr: MPU6050.MPU6050_I2C_ADDRESS, reg: REGISTER): number {
        let valh = i2cRead(addr, reg);
        let vall = i2cRead(addr, reg + 1);
        let val3 = (valh << 8) + vall
        return val3
    }

    /**
	 *Read data from MPU6050 register
	 * @param reg  register of MPU6050; eg: 0, 15, 23
	*/
    function readWord2C(addr: MPU6050_I2C_ADDRESS, reg: REGISTER): number {
        let val4 = readWord(addr, reg)
        if (val4 > 0x8000) {
            return -((65535 - val4) + 1)
        } else {
            return val4
        }
    }



    /**
	 * 倾斜角度
	*/
    //% blockId=MPU6050_get_angle
    //% block="获取MPU6050 |%axis| 方向的倾斜角度"
    //% group="IIC Communication" weight=75
    export function getAngle(addr: MPU6050_I2C_ADDRESS, axis: AXIS): number {
        if (checkAddress(addr)) {
            switch (axis) {
                case AXIS.X:
                    return Math.acos(getAccel(addr, axis)) * 57.29577
                case AXIS.Y:
                    return Math.acos(getAccel(addr, axis)) * 57.29577
                case AXIS.Z:
                    return Math.acos(getAccel(addr, axis)) * 57.29577
                default:
                    return 0
            }
        }
        return 0
    }


    /**
	 * 获取加速度 单位g
	*/
    //% blockId=MPU6050_get_accel
    //% block="获取MPU6050 |%axis|方向的加速度"
    //% group="IIC Communication" weight=75
    export function getAccel(addr: MPU6050_I2C_ADDRESS, axis: AXIS): number {
        if (checkAddress(addr)) {
            switch (axis) {
                case AXIS.X:
                    return (readWord2C(addr, REGISTER.ACCEL_X) + X_ACCEL_OFFSET) / 16384.0
                case AXIS.Y:
                    return (readWord2C(addr, REGISTER.ACCEL_Y) + Y_ACCEL_OFFSET) / 16384.0
                case AXIS.Z:
                    return (readWord2C(addr, REGISTER.ACCEL_Z) + Z_ACCEL_OFFSET) / 16384.0
                default:
                    return 0
            }
        }
        return 0
    }




    /**
	 * 获取角速度
	*/
    //% blockId=MPU6050_get_gyro
    //% block="获取MPU6050 |%axis|方向的角速度"
    //% group="IIC Communication" weight=75
    export function getGyro(addr: MPU6050_I2C_ADDRESS, axis: AXIS): number {
        if (checkAddress(addr)) {
            switch (axis) {
                case AXIS.X:
                    return (readWord2C(addr, REGISTER.GYRO_X) + X_GYRO_OFFSET)
                case AXIS.Y:
                    return (readWord2C(addr, REGISTER.GYRO_Y) + Y_GYRO_OFFSET)
                case AXIS.Z:
                    return (readWord2C(addr, REGISTER.GYRO_Z) + Z_GYRO_OFFSET)
                default:
                    return 0
            }
        }
        return 0
    }

    //% blockId="initialize_sensor" block="初始化颜色传感器"
    //% group="IIC Communication" 
    export function LCS_initialize() {
        // Make sure we're connected to the right sensor.
        let chip_id = I2C_ReadReg8(LCS_Constants.ADDRESS, (LCS_Constants.COMMAND_BIT | LCS_Constants.ID))

        if (chip_id != 0x44) {
            return // Incorrect chip ID
        }

        // Set default integration time and gain.
        LCS_set_integration_time(0.0048)
        LCS_set_gain(LCS_Constants.GAIN_16X)

        // Enable the device (by default, the device is in power down mode on bootup).
        LCS_enable()
    }

    function LCS_enable() {
        // Set the power and enable bits.
        I2C_WriteReg8(LCS_Constants.ADDRESS, (LCS_Constants.COMMAND_BIT | LCS_Constants.ENABLE), LCS_Constants.ENABLE_PON)
        basic.pause(10) // not sure if this is right    time.sleep(0.01) // FIXME delay for 10ms

        I2C_WriteReg8(LCS_Constants.ADDRESS, (LCS_Constants.COMMAND_BIT | LCS_Constants.ENABLE), (LCS_Constants.ENABLE_PON | LCS_Constants.ENABLE_AEN))
    }

    function LCS_set_integration_time(time: number) {
        let val = 0x100 - (time / 0.0024) // FIXME was cast to int type
        if (val > 255) {
            val = 255
        } else if (val < 0) {
            val = 0
        }
        I2C_WriteReg8(LCS_Constants.ADDRESS, (LCS_Constants.COMMAND_BIT | LCS_Constants.ATIME), val)
        LCS_integration_time_val = val
    }

    function LCS_set_gain(gain: number) {
        I2C_WriteReg8(LCS_Constants.ADDRESS, (LCS_Constants.COMMAND_BIT | LCS_Constants.CONTROL), gain)
    }


    function LCS_set_led_state(state: boolean) {
        I2C_WriteReg8(LCS_Constants.ADDRESS, (LCS_Constants.COMMAND_BIT | LCS_Constants.PERS), LCS_Constants.PERS_NONE)
        let val = I2C_ReadReg8(LCS_Constants.ADDRESS, (LCS_Constants.COMMAND_BIT | LCS_Constants.ENABLE))
        if (state) {
            val |= LCS_Constants.ENABLE_AIEN
        } else {
            val &= ~LCS_Constants.ENABLE_AIEN
        }
        I2C_WriteReg8(LCS_Constants.ADDRESS, (LCS_Constants.COMMAND_BIT | LCS_Constants.ENABLE), val)

        basic.pause(2 * (256 - LCS_integration_time_val) * 2.4) // delay for long enough for there to be new (post-change) complete values available
    }



    //% blockId="getSensorData" block="读取颜色值 %colorId"
    //% group="IIC Communication" 
    export function getColorData(color: RGB): number {
        basic.pause((256 - LCS_integration_time_val) * 2.4);
        let sum = I2C_ReadReg16(LCS_Constants.ADDRESS, (LCS_Constants.COMMAND_BIT | LCS_Constants.CDATAL));
        let vue = 0;
        switch (color) {
            case RGB.RED:
                vue = I2C_ReadReg16(LCS_Constants.ADDRESS, (LCS_Constants.COMMAND_BIT | LCS_Constants.RDATAL));

                break;
            case RGB.GREEN:
                vue = I2C_ReadReg16(LCS_Constants.ADDRESS, (LCS_Constants.COMMAND_BIT | LCS_Constants.GDATAL));

                break;
            case RGB.BLUE:
                vue = I2C_ReadReg16(LCS_Constants.ADDRESS, (LCS_Constants.COMMAND_BIT | LCS_Constants.BDATAL));

                break;
            case RGB.CLEAR:
                return sum;
                break;

        }
        vue = Math.floor(vue / sum * 255);

        serial.writeLine("val: " + vue);
        return vue;
    }


    function LCS_get_raw_data(delay: boolean = false): number[] {
        if (delay) {
            // Delay for the integration time to allow reading immediately after the previous read.
            basic.pause((256 - LCS_integration_time_val) * 2.4)
        }

        let div = (256 - LCS_integration_time_val) * 1024
        let rgbc = [0, 0, 0, 0]
        rgbc[0] = I2C_ReadReg16(LCS_Constants.ADDRESS, (LCS_Constants.COMMAND_BIT | LCS_Constants.RDATAL)) / div
        rgbc[1] = I2C_ReadReg16(LCS_Constants.ADDRESS, (LCS_Constants.COMMAND_BIT | LCS_Constants.GDATAL)) / div
        rgbc[2] = I2C_ReadReg16(LCS_Constants.ADDRESS, (LCS_Constants.COMMAND_BIT | LCS_Constants.BDATAL)) / div
        rgbc[3] = I2C_ReadReg16(LCS_Constants.ADDRESS, (LCS_Constants.COMMAND_BIT | LCS_Constants.CDATAL)) / div
        if (rgbc[0] > 1) {
            rgbc[0] = 1
        }
        if (rgbc[1] > 1) {
            rgbc[1] = 1
        }
        if (rgbc[2] > 1) {
            rgbc[2] = 1
        }
        if (rgbc[3] > 1) {
            rgbc[3] = 1
        }
        return rgbc
    }

}
