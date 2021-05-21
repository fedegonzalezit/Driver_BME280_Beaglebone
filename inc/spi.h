
#include <linux/init.h>           // Macros para funciones ej. __init __exit
#include <linux/module.h>         // Core header para carga de LKMs en el kernel
#include <linux/device.h>         // Header soporte del kernel Driver Model
#include <linux/kernel.h>         // types, macros, y funciones para el kernel
#include <linux/fs.h>             // Header para soporte del filesys Linux
#include <linux/interrupt.h>      // soporte de uso de IRQ
//#include <asm/uaccess.h>          // requerido para la funcion de copia al usuario
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/version.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/irqreturn.h>
#include <linux/syscalls.h>
#include <linux/types.h>
#include <linux/time.h>


#define  DEVICE_NAME "spi-td3"  ///< el dispositivo aparece con este nombre en /dev
#define  CLASS_NAME  "fsspi-bmp"      ///< nombre de la clase de dispositivo en el sysfs


//Macros para SPI0



#define SPI0_MODULE             0x48030000
#define CONTROL_MODULE          0x44E10000
#define SP0_CFG                 0x48038000
    
#define SPI0_CLK                0x950
#define SPI0_D0                 0x954
#define SPI0_D1                 0x958
#define SPI0_CS0                0x95C
#define SPI0_CS1                0x960

#define MCSPI_SYSCONFIG         0x110
#define MCSPI_SYSSTATUS         0x114
#define MCSPI_IRQSTATUS         0x118
#define MCSPI_IRQENA            0x11c
#define MCSPI_MODULCTRL         0x128
#define MCSPI_CH0_CONF          0x12C          
#define MCSPI_CH0CTRL           0x134
#define MCSPI_TX0   0x138
#define MCSPI_RX0   0x13C
#define MCSPI_CH0_CONF_WL_8     (0x07<<7)
#define MCSPI_CH0_CONF_WL_16     (0xF<<7)
#define MCSPI_CH0_CONF_CLK_32     (0x5<<2)
#define MCSPI_CH0_CONF_CLK_128     (0x7<<2)
#define MCSPI_CH0_CONF_CLK_32768     (0xf<<2)
#define	MCSPI_CH0_CONF_TS_3		(0x3<<25)
#define	MCSPI_CH0_CONF_TS_1		(0x1<<25)
#define MCSPI_SPIEN0_ENABLE		0x1
#define MCSPI_SPIEN0_DISABLE	~0x1
#define SPIDATDIR0_OUTPUT	~(1<<8)


#define CM_PER                  0x44E00000
#define CM_PER_SPI0_CLKCTRL     0x4c
#define OFFSET_CM_L4LS          0x00
#define OFFSET_SYST 		0x124

#define SPI_GCL         (1<<25)

#define RX0_INT_FULL    (1<<2)
#define TX0_INT_EMPTY	0x1
#define TX0_INT_OVERFLOW	1<<1
#define EOW_RESET       (0x1<<17)

#define BIT_FORCE       (1<<20)
#define EPOL_CS_ON           (1<<6)
#define EPOL_CS_OFF           ~(1<<6)

#define IDLEST_FUNC             ~(0x3<<16)
#define MODULE_MODE_ENABLE      0x2

#define BMP_ctrl_meas   0xF4
#define BMP_config    0xF5
#define BMP_IIR_COEF_4    0x08
#define BMP_IIR_OFF   0x00
#define BMP_TSB_05MS    0x00
#define BMP_NORMAL_MODE   0x03
#define BMP_P_ULTRA_HIGH  0x14
#define BMP_T_x2    0x40
#define BMP_READ_FLAG   0x80
#define BMP_PRESS_LSB   0xF8
#define BMP_PRESS_MSB   0xF7
#define BMP_TEMP_LSB    0xFB
#define BMP_TEMP_MSB    0xFA

#define	TIMEOUT_100MS 	10*HZ 
#define BUFFER_RX_SIZE		16

#define SPI_MODO_NORMAL   0
#define MODO_TABLAS_CONFIG  1


// Prototipado de funciones a implementar en el driver -- debe estar antes de la definicion de estructura file operations
static int     dev_open(struct inode *, struct file *);
static int     dev_release(struct inode *, struct file *);
static ssize_t dev_read(struct file *, char *, size_t, loff_t *);
static int dev_probe(struct platform_device *pdev);
static int dev_remove(struct platform_device *pdev);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
static int dev_ioctl(struct inode *i, struct file *f, unsigned int cmd, unsigned long arg);
#else
static long dev_ioctl(struct file *f, unsigned int cmd, unsigned long arg);
#endif

int8_t spi_reg_write(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t spi_reg_read(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);

void delay_ms(uint32_t period_ms);


int8_t bme_init_sin_api (struct bme280_dev *dev);

int8_t bme_set_config_sin_api (struct bme280_dev *dev, uint8_t config, uint8_t ctrl_meas,uint8_t ctrl_hum);

int8_t bme280_get_sensor_data_sin_api(struct bme280_data *comp_data, struct bme280_dev *dev);