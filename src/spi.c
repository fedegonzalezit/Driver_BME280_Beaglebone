/**
 * @file   spi.c
 * @author Federico Gonzalez
 * @date   17 Nov 2019
 * @version 0.1
 * @brief   Diver SPI
 */

#include "../inc/spi.h"
#include "../inc/bme280.h"
#include "../inc/bme280_defs.h"



MODULE_LICENSE("GPL");            ///< Tipo de licencia
MODULE_AUTHOR("Federico Gonzalez");    ///< Autor, visible con modinfo
MODULE_DESCRIPTION("Driver SPI y BMP 280");  ///< Descripcion visible con modinfo
MODULE_VERSION("0.1");            ///< Numero de versionado




static int    majorNumber;                  ///< Almacena el numero mayor de device, se determina automaticamente en este ejemplo
static struct class*  bmpClass  = NULL; 	  ///< puntero a device-driver class struct 
static struct device* bmpDevice = NULL; 	 ///< puntero a device-driver device struct
static struct cdev *my_cdev = NULL;
static dev_t  num_dispo;
static int virq;

static void  * regCM_PER = NULL;
static void  * regControl = NULL;
static void  * regSpi0 = NULL;
static void  * spi0_Config = NULL;
static char irq_condition = 0;
static char irq_txempty_condition = 0;
static uint8_t irq_txoverflow = 0;

struct bme280_dev dev;
int8_t rslt = BME280_OK; 
uint8_t settings_sel;
struct bme280_data comp_data;

uint32_t buffer_rx[BUFFER_RX_SIZE];
unsigned char buffer_rx_index_in = 0;
unsigned char buffer_rx_index_out = 0;


static uint32_t contadorDeMediciones = 0; //Requerimiento para primer recuperatorio
static DECLARE_WAIT_QUEUE_HEAD(queue_espera);
static DECLARE_WAIT_QUEUE_HEAD(queue_espera_rx);



/** @brief Estructura de definicion de funciones a implementar
 *  definida en  /linux/fs.h 
 *  usualmente solo implementamos open() read() write() y release()
 */
static struct file_operations fops =
{
   .owner = THIS_MODULE,
   .open = dev_open,	
   .read = dev_read,	
   .release = dev_release, // apaga todo 
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
    .ioctl = dev_ioctl,
#else
    .unlocked_ioctl = dev_ioctl,
#endif
};

  
static const struct of_device_id Mis_dispositivos_compatibles[] = 
{
  { .compatible = "BMP280-SPI", },
  {} 
};


static struct platform_driver My_SPI_Controller = {
    .probe = dev_probe,
    .remove = dev_remove,
    .driver = {
      .name =  DEVICE_NAME,
      .owner = THIS_MODULE,
      .of_match_table = of_match_ptr(Mis_dispositivos_compatibles),
    },
};



/** @brief Funcion de inicializacion del LKM
 *  El "static" restringe la visibilidad de la funcion dentro de este fuente. La macro __init
 *  entiende que para un driver built-in (no un LKM) la funcion solo se usa para el momento de inicializacion,
 *  puede ser descartada y su memoria liberada luego de este punto
 *  @return retorna 0 si esta OK
 */
static int __init spi_bmp_init(void){
  int err;

   printk(KERN_INFO "Driver spi: Inicializando Driver SPI...\n");



  err = alloc_chrdev_region(&num_dispo, 0, 1, DEVICE_NAME);
  if (err < 0) {
    printk(KERN_INFO "Major number allocation is failed\n");
    return err; 
  }
  majorNumber = MAJOR(num_dispo);
  printk(KERN_INFO "The major number is %d",majorNumber); 

  my_cdev = cdev_alloc( ); //Reservar memoria para la estructura cdev del kernel

  my_cdev->ops = &fops;
  my_cdev->owner = THIS_MODULE;
  //cdev_init(my_cdev , &fops);

  //Asociar la estructura cdev para el dispositivo y el número de dispositivo
  err = cdev_add(my_cdev,num_dispo, 1);
  if (err < 0)
  {
      printk(KERN_NOTICE "Error %d agregando dispositivo", err);
  }

  bmpClass = class_create(THIS_MODULE, CLASS_NAME);
  if (IS_ERR(bmpClass)){                // chequeo de error y cleanup si falla
      unregister_chrdev(majorNumber, DEVICE_NAME);
      printk(KERN_ALERT "Failed to register device class\n");
      return PTR_ERR(bmpClass);          // forma correcta de devolver un puntero como error
  }

  bmpDevice = device_create(bmpClass, NULL, num_dispo, NULL, DEVICE_NAME);
  if (IS_ERR(bmpDevice)){               // chequeo de error y cleanup si falla
      class_destroy(bmpClass);           // este codigo se repite pero la alternativa es hacer goto....en fin
      unregister_chrdev(majorNumber, DEVICE_NAME);
      printk(KERN_ALERT "falla al crear el dispositivo\n");
      return PTR_ERR(bmpDevice);
  }

  platform_driver_register(&My_SPI_Controller);

   printk(KERN_INFO "Driver spi: dispositivo creado correcatmente\n"); // inicializado OK!
   return 0;
}

/** @brief Funcion de remocion del LKM
 *  Similar a la de inicializacion, tambien static. La macro __exit notifica que si este 
 *  codigo es utilizado por un driver built-in (no un LKM) esta funcion no es requerida.
 */
static void __exit spi_bmp_exit(void){

   cdev_del(my_cdev);
   device_destroy(bmpClass, num_dispo); 
   class_destroy(bmpClass);                             // remuevo la clase del dispositivo
   unregister_chrdev_region(num_dispo, 1);
   platform_driver_unregister(&My_SPI_Controller);
   printk(KERN_ALERT "driver SPI: dispositivo desinstalado OK!\n");
}


/** @brief Funcion open
 *  Inicializa el dispositivo SPI y el BMP280
 */

static int dev_open(struct inode *inodep, struct file *filep){
  
  unsigned int reg;

  printk(KERN_INFO "Entro al open\n"); // inicializado OK!

  
  //Inicializo pines



    reg = ioread32(regControl + SPI0_CLK);   //PINMUX CLOCK
    iowrite32(reg & ~(0x4F), regControl + SPI0_CLK);
    reg = ioread32(regControl + SPI0_CLK);
    iowrite32(reg | 0x30, regControl + SPI0_CLK);    //Con RX_Active
    
    reg = ioread32(regControl + SPI0_D0);   //PINMUX D0
    iowrite32(reg & ~(0x6F),regControl + SPI0_D0);
    reg = ioread32(regControl + SPI0_D0);
    iowrite32(reg | 0x10, regControl + SPI0_D0);
    
    reg = ioread32(regControl + SPI0_D1);   //PINMUX D1
    iowrite32(reg & ~(0x4F), regControl + SPI0_D1);
    reg = ioread32(regControl + SPI0_D1);
    iowrite32(reg | 0x30, regControl + SPI0_D1);    //Con RX_Active
    
    reg = ioread32(regControl + SPI0_CS0);   //PINMUX CS0
    iowrite32(reg & ~(0x6F),regControl + SPI0_CS0);
    reg = ioread32(regControl + SPI0_CS0);
    iowrite32(reg | 0x10, regControl + SPI0_CS0);
    
    reg = ioread32(regControl + SPI0_CS1);   //PINMUX CS1
    iowrite32(reg & ~(0x6F),regControl + SPI0_CS1);
    reg = ioread32(regControl + SPI0_CS1);
    iowrite32(reg | 0x10, regControl + SPI0_CS1);


  //I. Habilitamos el modulo SPI


  reg = ioread32(regCM_PER + CM_PER_SPI0_CLKCTRL);
  reg = reg | MODULE_MODE_ENABLE;   // ENABLE
  iowrite32(reg, regCM_PER + CM_PER_SPI0_CLKCTRL);

  reg = ioread32(regCM_PER + CM_PER_SPI0_CLKCTRL);
  reg = reg & IDLEST_FUNC;         //Habilita IDLEST
  iowrite32(reg, regCM_PER + CM_PER_SPI0_CLKCTRL);
  reg = ioread32(regCM_PER + CM_PER_SPI0_CLKCTRL);

  printk(KERN_INFO "cmp_per_spi0_clkctrl: %x\n", reg);
  if (reg != 0x2)
  {
    printk(KERN_ALERT "error al activar el modulo SPI!, se leyo: %x\n", reg);
    return -1;
  }
  

    //II. Soft reset
  reg = ioread32(regSpi0 + MCSPI_SYSCONFIG);
  iowrite32((reg | 1<<1), regSpi0 + MCSPI_SYSCONFIG);
 

  reg = ioread32(regCM_PER + OFFSET_CM_L4LS);
  printk(KERN_INFO "reg L4LS: %x\n", reg);

  udelay(100);
  //III. Verificar si el reset se cumplió (Leer MCSPI_SYSSTATUS)
  if ((ioread32(regSpi0 + MCSPI_SYSSTATUS)&1) != 1)
  {
    printk(KERN_ALERT "No se termino de ejecutar el reset\n"); 
    return PTR_ERR(0);
  }
  printk(KERN_INFO "Se reseteo correctamente:\n");
  
      
  //IV. Configuración del modulo SPI:
    //a. Escribimos en MCSPI_MODULCTRL
  iowrite32 (0x0 , regSpi0 + MCSPI_MODULCTRL);
  printk(KERN_INFO "MCSPI_MODULCTRL: %x:\n", ioread32(regSpi0 + MCSPI_MODULCTRL));

    //b. Escribimos 0x308 en MCSPI_SYSCONFIG
  iowrite32(0x308, regSpi0 + MCSPI_SYSCONFIG);
  printk(KERN_INFO "MCSPI_SYSCONFIG: %x:\n", ioread32(regSpi0 + MCSPI_SYSCONFIG));



  //V. Inicializar Interrupción
    //a. Reset status bits MCSPI_IRQSTATUS
  //reg = ioread32(regSpi0 + OFFSET_SYST);
  //iowrite32 (reg | (1<<11), regSpi0 + OFFSET_SYST ); //<< poner en 1 este bit limpia todo el irq status
  iowrite32 (0xFFFFFFFF, regSpi0 + MCSPI_IRQSTATUS);
  //iowrite32((reg | EOW_RESET | 0x07),regSpi0 + MCSPI_IRQSTATUS);
  reg = ioread32(regSpi0 + MCSPI_IRQSTATUS);
  printk (KERN_INFO "interrupciones status %x\n", reg);


    //b. Habilitar interrupciones en MCSPI_IRQENA
  //reg = ioread32(regSpi0 + MCSPI_IRQENA);


  iowrite32 (TX0_INT_OVERFLOW | TX0_INT_EMPTY | RX0_INT_FULL | 1<<3, regSpi0 + MCSPI_IRQENA); 

  //iowrite32(reg | 0x1<<2 | 0x1,regSpi0 + MCSPI_IRQENA);
  //iowrite32(0x5,regSpi0 + MCSPI_IRQENA);
  
  //iowrite32(reg | 0x5,regSpi0 + MCSPI_IRQENA);
  reg = ioread32(regSpi0 + MCSPI_IRQENA);
  printk (KERN_INFO "registro de interrupciones %x\n", reg);


  //VI. Configurar el canal que se utilizará: MCSPI_CH0_CONF

  reg = ioread32(regSpi0+MCSPI_CH0_CONF);


  //iowrite32(0x100103df,regSpi0+MCSPI_CH0_CONF);
  iowrite32(0x00060000 | (MCSPI_CH0_CONF_WL_16) | 0x9<<2 | EPOL_CS_ON | BIT_FORCE | 0x3, regSpi0 + MCSPI_CH0_CONF);
  //iowrite32( 0x00060000 | MCSPI_CH0_CONF_WL_8 |  MCSPI_CH0_CONF_CLK_32  | EPOL_CS_ON, regSpi0+MCSPI_CH0_CONF);
  reg = ioread32(regSpi0+MCSPI_CH0_CONF);
  printk (KERN_INFO "MCSPI_CH0_CONF: %x\n", reg);

  //Habilito para que se pueda manejar el SPIEN (CS)
  reg = ioread32(regSpi0 + OFFSET_SYST);
  iowrite32 (reg & SPIDATDIR0_OUTPUT, regSpi0 + OFFSET_SYST);



  //VII. Iniciar/Controlar el canal en MCSPI_CH(i)CTRL
  reg = ioread32(regSpi0 + MCSPI_CH0CTRL);
  iowrite32(reg | 0x1, regSpi0 + MCSPI_CH0CTRL);



  printk (KERN_INFO "Spi inicializado \n");



  spi_reg_read(0, 0xf3, &rslt, 1); //Linea de prueba para ver si hay algun problema con este registro o qe
  //Inicialización del BME (USANDO API)


  /* Sensor_0 interface over SPI with native chip select line */
  dev.dev_id = 0;
  dev.intf = BME280_SPI_INTF;
  dev.read = spi_reg_read;
  dev.write = spi_reg_write;
  dev.delay_ms = delay_ms;

  rslt = bme280_init(&dev);


  printk(KERN_INFO "bmp280_init status %d\n", rslt);  
  if (rslt == BME280_OK)
  {
      printk (KERN_INFO "Inicializa el bmp OK\n");
  }
  else
  {
      printk (KERN_INFO "Error al inicializar el bmp\n");
      //return rslt;
  }





  /* Modo de operación recomendado segun manual: Indoor navigation */
  dev.settings.osr_h = BME280_OVERSAMPLING_1X;
  dev.settings.osr_p = BME280_OVERSAMPLING_16X;
  dev.settings.osr_t = BME280_OVERSAMPLING_2X;
  dev.settings.filter = BME280_FILTER_COEFF_16;
  dev.settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

  settings_sel = BME280_OSR_PRESS_SEL;
  settings_sel |= BME280_OSR_TEMP_SEL;
  settings_sel |= BME280_OSR_HUM_SEL;
  settings_sel |= BME280_STANDBY_SEL;
  settings_sel |= BME280_FILTER_SEL;
  rslt = bme280_set_sensor_settings(settings_sel, &dev);
  rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev);


  
  printk(KERN_ALERT "driver SPI: Dispositivo abierto correctamente\n");
  return 0;
}


// Llamada por read()
static ssize_t dev_read(struct file *filep, char *buffer, size_t len, loff_t *offset){
  char *buff_kernel = NULL;
  int err;



  err = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);

  contadorDeMediciones++;
/*
  printk (KERN_INFO "Se leyo, Temperatura: %d\n", comp_data.temperature);
  printk (KERN_INFO "Se leyo, Presion: %d\n", comp_data.pressure);
  printk (KERN_INFO "Se leyo, HUmedad: %d\n", comp_data.humidity);
 */




  buff_kernel = kmalloc(12*sizeof(char), GFP_KERNEL);

  buff_kernel[0] = (char)(comp_data.temperature & 0xFF);
  buff_kernel[1] = (char)((comp_data.temperature>>8) & 0xFF);
  buff_kernel[2] = (char)((comp_data.temperature>>16) & 0xFF);
  buff_kernel[3] = (char)((comp_data.temperature>>24) & 0xFF);
  buff_kernel[4] = (char)(comp_data.pressure & 0xFF);
  buff_kernel[5] = (char)((comp_data.pressure>>8) & 0xFF);
  buff_kernel[6] = (char)((comp_data.pressure>>16) & 0xFF);
  buff_kernel[7] = (char)((comp_data.pressure>>24) & 0xFF);
  buff_kernel[8] = (char)(comp_data.humidity & 0xFF);
  buff_kernel[9] = (char)((comp_data.humidity>>8) & 0xFF);
  buff_kernel[10] = (char)((comp_data.humidity>>16) & 0xFF);
  buff_kernel[11] = (char)((comp_data.humidity>>24) & 0xFF);

  

  if (len > 12)
  {
    len = 12;
  }

  if ((err = copy_to_user(buffer, buff_kernel, len)) < 0)
  {
    printk(KERN_ERR "Error al copiar datos a usuario \n");
    kfree(buff_kernel);
    return -2;
  }

//  printk(KERN_ALERT "Copy to user ok\n");
  kfree(buff_kernel);

  return err;


}



// la funcion que llama el usuario para "cerrar" el dispositivo
static int dev_release(struct inode *inodep, struct file *filep){

int aux;

  iowrite32(0x0, regSpi0 + MCSPI_IRQENA); //Deshabilito la interrupcion
  aux = ioread32 (regSpi0 + MCSPI_CH0CTRL); //Deshabilito el canal
  iowrite32 (aux & (~(0x1)), regSpi0 + MCSPI_CH0CTRL);
  //Aca tengo que apagar el canal y desactivar interrupciones
  return 0;
}


//La onda es en esta funcion configurar el driver y configurar si quiero recivir toda la tabla de digitos
// para la configuracion
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
static int dev_ioctl(struct inode *i, struct file *f, unsigned int cmd, unsigned long arg)
#else
static long dev_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
#endif
{

 
  return contadorDeMediciones;
}




static irqreturn_t dev_spi_handler_ISR (int irq, void *dev_id)
{

  static int reg;


  reg = ioread32(regSpi0 + MCSPI_IRQSTATUS);


  if (reg & RX0_INT_FULL)
  {
    
    //  printk (KERN_INFO "Entro a RX0_INT_FULL\n");

      if (irq_condition == 0)
      {
          irq_condition = 1;  //Condición para despertar la tarea
          wake_up_interruptible(&queue_espera_rx);
      }

      iowrite32 (RX0_INT_FULL ,regSpi0 + MCSPI_IRQSTATUS);  //Limpio la interrupcion


  }

  if ((reg & TX0_INT_EMPTY))
  {
      //printk(KERN_ALERT "Interrupción tx vacio\n");
      //printk (KERN_INFO "Entro a TX0_INT_EMPTY\n");

      if (irq_txempty_condition == 0)
      {
            irq_txempty_condition = 1;  //Condición para despertar la tarea
          wake_up_interruptible(&queue_espera);  
      }

      iowrite32 (TX0_INT_EMPTY ,regSpi0 + MCSPI_IRQSTATUS);  //Limpio la interrupcion

  }

  if (reg & (1<<3))
  {
    //printk (KERN_INFO "RX0 OVERFLOW");
    iowrite32 (reg | 1<<3, regSpi0 + MCSPI_IRQSTATUS);
  }

  if (reg & TX0_INT_OVERFLOW)
  {
    iowrite32 (reg | TX0_INT_OVERFLOW, regSpi0 + MCSPI_IRQSTATUS);
    irq_txoverflow = 1;

  }
  
  return (irqreturn_t) IRQ_HANDLED;
  
}


static int dev_probe(struct platform_device *pdev)
{
static int err;

      contadorDeMediciones = 0;
      printk(KERN_INFO "Entro al probe\n"); 
      virq = platform_get_irq (pdev, 0); //obtener una línea de interrupción disponible

      if (virq < 0)
      {
          pr_err("IRQ not availabel\n");
          return -1;
      }
      printk(KERN_INFO "get irq ok\n"); 

      err = request_irq(virq,dev_spi_handler_ISR, IRQF_TRIGGER_RISING, pdev->name, NULL);
      if (err < 0)
      {
          pr_err("handler irq error\n");
          return -1;
      }
      printk(KERN_INFO "request irq ok err=%d\n", err); 

      if ((regCM_PER = ioremap (CM_PER, 0x1000)) == NULL)   //Paginacion de una pagina para CM_PER
      {
          printk(KERN_ERR "Mapping ROM failed\n");
          return -1;
      }
      printk(KERN_INFO "ioremap cm_per ok\n"); 

      if ((regControl = ioremap (CONTROL_MODULE, 0x1000)) == NULL)
      {
          printk(KERN_ERR "Mapping ROM failed\n");
          return -1;
      }
      printk(KERN_INFO "ioremap reg_control ok\n"); 

      if ((regSpi0 = ioremap (SPI0_MODULE, 0x1000)) == NULL)
      {
          printk(KERN_ERR "Mapping ROM failed\n");
          return -1;
      }    
      printk(KERN_INFO "ioremap regSpi ok\n"); 

      
      if ((spi0_Config = ioremap (SP0_CFG, 0x1000)) == NULL)
      {
          printk(KERN_ERR "Mapping ROM failed\n");
          return -1;
      }    
      printk(KERN_INFO "ioremap spi0_Config ok\n"); 


      printk(KERN_ALERT "Probe() ok\n");

      return 0;

}

static int dev_remove(struct platform_device *pdev)
{
    free_irq(virq, NULL);
    iounmap(regCM_PER);
    iounmap(regControl);
    iounmap (regSpi0);
    iounmap (spi0_Config);

    printk(KERN_ALERT "Se removio\n");

    return 0;
}

// Un modulo debe usar las macros module_init() module_exit() de linux/init.h, 
//  que identifican las funciones de instalacion y remocion 
module_init(spi_bmp_init);
module_exit(spi_bmp_exit);
MODULE_DEVICE_TABLE(of, Mis_dispositivos_compatibles);

//--------------------------------------------------------------------------------------------///
//------------------------------------Funciones auxiliares------------------------------------///
//---------------------------------------------------------------------------------------------//

int8_t spi_reg_write(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
  int i;
  unsigned int reg_spien;
  int lectura;
  uint16_t tx;



 //SPI EN (CS)
  reg_spien = ioread32 (regSpi0 + OFFSET_SYST); 
  iowrite32 (reg_spien | MCSPI_SPIEN0_ENABLE, regSpi0 + OFFSET_SYST);


  for (i=0; i<length; i++)
  {
    tx = ((reg_addr+i)<<8) | reg_data[i];
    irq_condition = 0;
   // printk (KERN_INFO "Esctribió, Transmitió, espera int tx empty: %x\n", tx);
    iowrite32 (tx, regSpi0 + MCSPI_TX0);
    wait_event_interruptible (queue_espera_rx, irq_condition==1);


    lectura = ioread32(regSpi0 + MCSPI_RX0);  //Si no leo despues de RX full se rompe todo.
 //   printk (KERN_INFO "Termino wait de rx full, recibio: %x\n", lectura);
  //  printk (KERN_INFO "Salio del wait\n");

  }

  //SPI_DISABLE
  iowrite32 (reg_spien & MCSPI_SPIEN0_DISABLE, regSpi0 + OFFSET_SYST);
  return 0;

}

int8_t spi_reg_read(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
  int i;
  unsigned int reg_spien;
  int lectura;
  int aux;

  //SPI EN (CS)

  reg_spien = ioread32 (regSpi0 + OFFSET_SYST);
  iowrite32 (reg_spien | MCSPI_SPIEN0_ENABLE, regSpi0 + OFFSET_SYST);


   for (i=0; i<length; i++)
  {
      aux = (reg_addr+i)<<8 ;
     // printk (KERN_INFO "Se envió para leer: %x\n", aux);
      irq_condition = 0;
      iowrite32 (aux, regSpi0 + MCSPI_TX0);

    //  printk (KERN_INFO "Espera interrupcion\n");
      if (irq_condition == 0)
      {
          wait_event_interruptible (queue_espera_rx, irq_condition==1);
      }


      lectura = ioread32(regSpi0 + MCSPI_RX0);
    //  printk (KERN_INFO "Termino wait de rx full, recibio: %x\n", lectura);
      reg_data[i] = (lectura & 0xFF);


  }

  

  //SPI_DISABLE
  iowrite32 (reg_spien & MCSPI_SPIEN0_DISABLE, regSpi0 + OFFSET_SYST);

  return 0;

}


/*!
 *  @brief Function that creates a mandatory delay required in some of the APIs such as "bmg250_soft_reset",
 *      "bmg250_set_foc", "bmg250_perform_self_test"  and so on.
 *
 *  @param[in] period_ms  : the required wait time in milliseconds.
 *  @return void.
 *
 */
void delay_ms(uint32_t period_ms)
{
  msleep(period_ms);
}




/*
* Funciones básicas para que funcione el BME sin usar la  API
* Para hacer test use la API, no se si se puede entregar así. por las dudas entrego con estas funciones hechas por mí 
*/

/*!
 * @brief This internal API reads the calibration data from the sensor, parse
 * it and store in the device structure.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t get_calib_data(struct bme280_dev *dev);


/*
*Funcion que chequea que se pueda leer el ID y hace un soft reset
* Toma los datos de calibración pero esa parte lo hago con la API. 
* (El manual recomienda fuertemente usar la api para la configuración)
*/
int8_t bme_init_sin_api (struct bme280_dev *dev)
{
  uint8_t response;
  uint8_t data;

  response = dev->read (0, 0xD0, &data, 1);  //Lee ID
  if (data == 0x60)
  {

    data = 0xB6;

    //Escribe el comando para realizar un soft reset, el primer bit debe ser 0, por eso la mascara
    dev->write (0, 0xE0 & (~(1<<8)), &data, 1);  

    dev->delay_ms(2); //Hace delay de 2ms, es lo que tarda en completar el reset

    get_calib_data(dev); //Toma los datos de calibración, esta parte sí la hago con la api

    return 0;
  }
  return -1;

}

/*
* Realiza la configuración en el orden que sugiere el manual
*/
int8_t bme_set_config_sin_api (struct bme280_dev *dev, uint8_t config, uint8_t ctrl_meas,uint8_t ctrl_hum)
{

  uint8_t data;

  //Primero leo el modo, segun el manual si no está en sleep mode la configuración puede no realizarse
  dev->read (0, 0xF4, &data, 1);

  if ((data & 0x3) != 0x00)
  {
    //Si los primeros 2 bits de data no son 00 no está en sleep mode. Lo tengo que pasar a este modo escribir config
    dev->write(0, 0xF4 & (~(1<<8)), 0x00, 1);  //Lo paso a sleep mode.
  }

  //Este orden de escritura lo recomienda el manual

  //Escribe en el registro de config
  dev->write(0, 0xF5& (~(1<<8)), &config, 1);

  //Escribe en el registro crl_meas
  ctrl_meas &= ~0x3;
  dev->write(0, 0xF4&  (~(1<<8)), &ctrl_meas, 1);  //La mascara está para que no se ponga en modo normal  

  //escribe en el registro crl_hum
  dev->write(0, 0xF2&  (~(1<<8)), &ctrl_hum, 1);

  //Seteo el modo de funcionamiento normal
  ctrl_meas |= 0x3;
  dev->write(0, 0xF4&  (~(1<<8)), &ctrl_meas, 1); //Con la mascara en 0x3 lo paso a modo normal.
  return 0;
}






/*
* Lee datos del BME y hace la compensación con la api
*/
int8_t bme280_get_sensor_data_sin_api(struct bme280_data *comp_data, struct bme280_dev *dev)
{
  struct  bme280_uncomp_data ucomp_data;
  uint32_t utemp;
  uint32_t upress;
  uint32_t uhum;
  uint8_t data_buffer[3];


  //Lee los datos de temperatura
  //Tiene que leer los registros 0xFA FB Y FC, al pasar el length 3 la funcion read sabe que lee esos 3 registros
  dev->read(0, 0xFA , data_buffer, 3);

  //Recibe, temperatura msb, temperatura lsb y temperatura xlsb (en ese orden)
  utemp = data_buffer[0];
  utemp = (utemp<<8) | data_buffer[1];
  utemp = (utemp<<4) | (data_buffer[2]>>4);

  //Idem con presion y humedad
  dev->read(0, 0xF7, data_buffer, 3);
  upress = data_buffer[0];
  upress = (upress<<8) | data_buffer[1];
  upress = (upress<<4) | (data_buffer[2]>>4);
  //Humedad son solo 2 bytes
  dev->read(0, 0xF7, data_buffer, 2);
  uhum = data_buffer[0];
  uhum = (uhum<<8) | (data_buffer[1]);


  ucomp_data.temperature = utemp;
  ucomp_data.pressure = upress;
  ucomp_data.humidity =uhum;

  //Compensa la medición
  bme280_compensate_data(BME280_ALL, &ucomp_data, comp_data, &dev->calib_data);


  return 0;
}


/* A PARTIR DE ACA ARRANCA LA API DE BME280. */



/**\name Internal macros */
/* To identify osr settings selected by user */
#define OVERSAMPLING_SETTINGS   UINT8_C(0x07)

/* To identify filter and standby settings selected by user */
#define FILTER_STANDBY_SETTINGS UINT8_C(0x18)

/*!
 * @brief This internal API puts the device to sleep mode.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 *
 * @return Result of API execution status.
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t put_device_to_sleep(const struct bme280_dev *dev);

/*!
 * @brief This internal API writes the power mode in the sensor.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 * @param[in] sensor_mode : Variable which contains the power mode to be set.
 *
 * @return Result of API execution status.
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t write_power_mode(uint8_t sensor_mode, const struct bme280_dev *dev);

/*!
 * @brief This internal API is used to validate the device pointer for
 * null conditions.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t null_ptr_check(const struct bme280_dev *dev);

/*!
 * @brief This internal API interleaves the register address between the
 * register data buffer for burst write operation.
 *
 * @param[in] reg_addr : Contains the register address array.
 * @param[out] temp_buff : Contains the temporary buffer to store the
 * register data and register address.
 * @param[in] reg_data : Contains the register data to be written in the
 * temporary buffer.
 * @param[in] len : No of bytes of data to be written for burst write.
 */
static void interleave_reg_addr(const uint8_t *reg_addr, uint8_t *temp_buff, const uint8_t *reg_data, uint8_t len);



/*!
 *  @brief This internal API is used to parse the temperature and
 *  pressure calibration data and store it in the device structure.
 *
 *  @param[out] dev : Structure instance of bme280_dev to store the calib data.
 *  @param[in] reg_data : Contains the calibration data to be parsed.
 */
static void parse_temp_press_calib_data(const uint8_t *reg_data, struct bme280_dev *dev);

/*!
 *  @brief This internal API is used to parse the humidity calibration data
 *  and store it in device structure.
 *
 *  @param[out] dev : Structure instance of bme280_dev to store the calib data.
 *  @param[in] reg_data : Contains calibration data to be parsed.
 */
static void parse_humidity_calib_data(const uint8_t *reg_data, struct bme280_dev *dev);


/*!
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in integer data type.
 *
 * @param[in] uncomp_data : Contains the uncompensated temperature data.
 * @param[in] calib_data : Pointer to calibration data structure.
 *
 * @return Compensated temperature data.
 * @retval Compensated temperature data in integer.
 */
static int32_t compensate_temperature(const struct bme280_uncomp_data *uncomp_data,
                                      struct bme280_calib_data *calib_data);

/*!
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in integer data type.
 *
 * @param[in] uncomp_data : Contains the uncompensated pressure data.
 * @param[in] calib_data : Pointer to the calibration data structure.
 *
 * @return Compensated pressure data.
 * @retval Compensated pressure data in integer.
 */
static uint32_t compensate_pressure(const struct bme280_uncomp_data *uncomp_data,
                                    const struct bme280_calib_data *calib_data);

/*!
 * @brief This internal API is used to compensate the raw humidity data and
 * return the compensated humidity data in integer data type.
 *
 * @param[in] uncomp_data : Contains the uncompensated humidity data.
 * @param[in] calib_data : Pointer to the calibration data structure.
 *
 * @return Compensated humidity data.
 * @retval Compensated humidity data in integer.
 */
static uint32_t compensate_humidity(const struct bme280_uncomp_data *uncomp_data,
                                    const struct bme280_calib_data *calib_data);


/*!
 * @brief This internal API is used to identify the settings which the user
 * wants to modify in the sensor.
 *
 * @param[in] sub_settings : Contains the settings subset to identify particular
 * group of settings which the user is interested to change.
 * @param[in] desired_settings : Contains the user specified settings.
 *
 * @return Indicates whether user is interested to modify the settings which
 * are related to sub_settings.
 * @retval True -> User wants to modify this group of settings
 * @retval False -> User does not want to modify this group of settings
 */
static uint8_t are_settings_changed(uint8_t sub_settings, uint8_t desired_settings);

/*!
 * @brief This API sets the humidity oversampling settings of the sensor.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t set_osr_humidity_settings(const struct bme280_settings *settings, const struct bme280_dev *dev);

/*!
 * @brief This internal API sets the oversampling settings for pressure,
 * temperature and humidity in the sensor.
 *
 * @param[in] desired_settings : Variable used to select the settings which
 * are to be set.
 * @param[in] dev : Structure instance of bme280_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t set_osr_settings(uint8_t desired_settings,
                               const struct bme280_settings *settings,
                               const struct bme280_dev *dev);

/*!
 * @brief This API sets the pressure and/or temperature oversampling settings
 * in the sensor according to the settings selected by the user.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 * @param[in] desired_settings: variable to select the pressure and/or
 * temperature oversampling settings.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t set_osr_press_temp_settings(uint8_t desired_settings,
                                          const struct bme280_settings *settings,
                                          const struct bme280_dev *dev);

/*!
 * @brief This internal API fills the pressure oversampling settings provided by
 * the user in the data buffer so as to write in the sensor.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 * @param[out] reg_data : Variable which is filled according to the pressure
 * oversampling data provided by the user.
 */
static void fill_osr_press_settings(uint8_t *reg_data, const struct bme280_settings *settings);

/*!
 * @brief This internal API fills the temperature oversampling settings provided
 * by the user in the data buffer so as to write in the sensor.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 * @param[out] reg_data : Variable which is filled according to the temperature
 * oversampling data provided by the user.
 */
static void fill_osr_temp_settings(uint8_t *reg_data, const struct bme280_settings *settings);

/*!
 * @brief This internal API sets the filter and/or standby duration settings
 * in the sensor according to the settings selected by the user.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 * @param[in] desired_settings : variable to select the filter and/or
 * standby duration settings.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t set_filter_standby_settings(uint8_t desired_settings,
                                          const struct bme280_settings *settings,
                                          const struct bme280_dev *dev);

/*!
 * @brief This internal API fills the filter settings provided by the user
 * in the data buffer so as to write in the sensor.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 * @param[out] reg_data : Variable which is filled according to the filter
 * settings data provided by the user.
 */
static void fill_filter_settings(uint8_t *reg_data, const struct bme280_settings *settings);

/*!
 * @brief This internal API fills the standby duration settings provided by the
 * user in the data buffer so as to write in the sensor.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 * @param[out] reg_data : Variable which is filled according to the standby
 * settings data provided by the user.
 */
static void fill_standby_settings(uint8_t *reg_data, const struct bme280_settings *settings);

/*!
 * @brief This internal API parse the oversampling(pressure, temperature
 * and humidity), filter and standby duration settings and store in the
 * device structure.
 *
 * @param[out] dev : Structure instance of bme280_dev.
 * @param[in] reg_data : Register data to be parsed.
 */
static void parse_device_settings(const uint8_t *reg_data, struct bme280_settings *settings);

/*!
 * @brief This internal API reloads the already existing device settings in the
 * sensor after soft reset.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 * @param[in] settings : Pointer variable which contains the settings to
 * be set in the sensor.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t reload_device_settings(const struct bme280_settings *settings, const struct bme280_dev *dev);

/****************** Global Function Definitions *******************************/

/*!
 *  @brief This API is the entry point.
 *  It reads the chip-id and calibration data from the sensor.
 */
int8_t bme280_init(struct bme280_dev *dev)
{
    int8_t rslt;

    /* chip id read try count */
    uint8_t try_count = 5;
    uint8_t chip_id = 0;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BME280_OK)
    {
        while (try_count)
        {
            /* Read the chip-id of bme280 sensor */
            rslt = bme280_get_regs(BME280_CHIP_ID_ADDR, &chip_id, 1, dev);

            /* Check for chip id validity */
            if ((rslt == BME280_OK) && (chip_id == BME280_CHIP_ID))
            {
                dev->chip_id = chip_id;

                /* Reset the sensor */
               rslt = bme280_soft_reset(dev); //
                if (rslt == BME280_OK)
                {
                    /* Read the calibration data */
                    rslt = get_calib_data(dev);
                }
                break;
            }

            /* Wait for 1 ms */
            dev->delay_ms(1);
            --try_count;
        }

        /* Chip id check failed */
        if (!try_count)
        {
            rslt = BME280_E_DEV_NOT_FOUND;
        }
    }

    return rslt;
}

/*!
 * @brief This API reads the data from the given register address of the sensor.
 */
int8_t bme280_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, const struct bme280_dev *dev)
{
    int8_t rslt;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BME280_OK)
    {
        /* If interface selected is SPI */
        if (dev->intf != BME280_I2C_INTF)
        {
            reg_addr = reg_addr | 0x80;
        }

        /* Read the data  */
        rslt = dev->read(dev->dev_id, reg_addr, reg_data, len);

        /* Check for communication error */
        if (rslt != BME280_OK)
        {
            rslt = BME280_E_COMM_FAIL;
        }
    }

    return rslt;
}

/*!
 * @brief This API writes the given data to the register address
 * of the sensor.
 */
int8_t bme280_set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len, const struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t temp_buff[20]; /* Typically not to write more than 10 registers */
    uint16_t temp_len;
    uint8_t reg_addr_cnt;

    if (len > 10)
    {
        len = 10;
    }


    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Check for arguments validity */
    if ((rslt == BME280_OK) && (reg_addr != NULL) && (reg_data != NULL))
    {
        if (len != 0)
        {
            temp_buff[0] = reg_data[0];

            /* If interface selected is SPI */
            if (dev->intf != BME280_I2C_INTF)
            {
                for (reg_addr_cnt = 0; reg_addr_cnt < len; reg_addr_cnt++)
                {
                    reg_addr[reg_addr_cnt] = reg_addr[reg_addr_cnt] & 0x7F;
                }
            }

            /* Burst write mode */
            if (len > 1)
            {
                /* Interleave register address w.r.t data for
                 * burst write
                 */
                interleave_reg_addr(reg_addr, temp_buff, reg_data, len);
                temp_len = ((len * 2) - 1);
            }
            else
            {
                temp_len = len;
            }
            rslt = dev->write(dev->dev_id, reg_addr[0], temp_buff, temp_len);

            /* Check for communication error */
            if (rslt != BME280_OK)
            {
                rslt = BME280_E_COMM_FAIL;
            }
        }
        else
        {
            rslt = BME280_E_INVALID_LEN;
        }
    }
    else
    {
        rslt = BME280_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API sets the oversampling, filter and standby duration
 * (normal mode) settings in the sensor.
 */
int8_t bme280_set_sensor_settings(uint8_t desired_settings, const struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t sensor_mode;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BME280_OK)
    {
        rslt = bme280_get_sensor_mode(&sensor_mode, dev);
        printk (KERN_INFO "Salió de leer el modo\n");

        if ((rslt == BME280_OK) && (sensor_mode != BME280_SLEEP_MODE))
        {
            printk (KERN_INFO "Entró a poner el dispositivo a dormir \n");
            rslt = put_device_to_sleep(dev);
        }
        if (rslt == BME280_OK)
        {
            /* Check if user wants to change oversampling
             * settings
             */
            if (are_settings_changed(OVERSAMPLING_SETTINGS, desired_settings))
            {
                printk (KERN_INFO "Entró aca 1 \n");

                rslt = set_osr_settings(desired_settings, &dev->settings, dev);
            }

            /* Check if user wants to change filter and/or
             * standby settings
             */
            if ((rslt == BME280_OK) && are_settings_changed(FILTER_STANDBY_SETTINGS, desired_settings))
            {
                printk (KERN_INFO "Entró aca 2 \n");

                rslt = set_filter_standby_settings(desired_settings, &dev->settings, dev);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API gets the oversampling, filter and standby duration
 * (normal mode) settings from the sensor.
 */
int8_t bme280_get_sensor_settings(struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data[4];

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BME280_OK)
    {
        rslt = bme280_get_regs(BME280_CTRL_HUM_ADDR, reg_data, 4, dev);
        if (rslt == BME280_OK)
        {
            parse_device_settings(reg_data, &dev->settings);
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the power mode of the sensor.
 */
int8_t bme280_set_sensor_mode(uint8_t sensor_mode, const struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t last_set_mode;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    if (rslt == BME280_OK)
    {
        rslt = bme280_get_sensor_mode(&last_set_mode, dev);

        /* If the sensor is not in sleep mode put the device to sleep
         * mode
         */
        if ((rslt == BME280_OK) && (last_set_mode != BME280_SLEEP_MODE))
        {
            rslt = put_device_to_sleep(dev);
        }

        /* Set the power mode */
        if (rslt == BME280_OK)
        {
            rslt = write_power_mode(sensor_mode, dev);
        }
    }

    return rslt;
}

/*!
 * @brief This API gets the power mode of the sensor.
 */
int8_t bme280_get_sensor_mode(uint8_t *sensor_mode, const struct bme280_dev *dev)
{
    int8_t rslt;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    if (rslt == BME280_OK)
    {
        /* Read the power mode register */
        rslt = bme280_get_regs(BME280_PWR_CTRL_ADDR, sensor_mode, 1, dev);

        /* Assign the power mode in the device structure */
        *sensor_mode = BME280_GET_BITS_POS_0(*sensor_mode, BME280_SENSOR_MODE);
    }

    return rslt;
}

/*!
 * @brief This API performs the soft reset of the sensor.
 */
int8_t bme280_soft_reset(const struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t reg_addr = BME280_RESET_ADDR;
    uint8_t status_reg = 0;
    uint8_t try_run = 5;

    /* 0xB6 is the soft reset command */
    uint8_t soft_rst_cmd = BME280_SOFT_RESET_COMMAND;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BME280_OK)
    {
        /* Write the soft reset command in the sensor */
        rslt = bme280_set_regs(&reg_addr, &soft_rst_cmd, 1, dev);

        if (rslt == BME280_OK)
        {
            /* If NVM not copied yet, Wait for NVM to copy */
            do
            {
                /* As per data sheet - Table 1, startup time is 2 ms. */
                dev->delay_ms(2);
                rslt = bme280_get_regs(BME280_STATUS_REG_ADDR, &status_reg, 1, dev);
            } while ((rslt == BME280_OK) && (try_run--) && (status_reg & BME280_STATUS_IM_UPDATE));

            if (status_reg & BME280_STATUS_IM_UPDATE)
            {
                rslt = BME280_E_NVM_COPY_FAILED;
            }

        }
    }

    return rslt;
}

/*!
 * @brief This API reads the pressure, temperature and humidity data from the
 * sensor, compensates the data and store it in the bme280_data structure
 * instance passed by the user.
 */
int8_t bme280_get_sensor_data(uint8_t sensor_comp, struct bme280_data *comp_data, struct bme280_dev *dev)
{
    int8_t rslt;

    /* Array to store the pressure, temperature and humidity data read from
     * the sensor
     */
    uint8_t reg_data[BME280_P_T_H_DATA_LEN] = { 0 };
    struct bme280_uncomp_data uncomp_data = { 0 };

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    if ((rslt == BME280_OK) && (comp_data != NULL))
    {
        /* Read the pressure and temperature data from the sensor */
        rslt = bme280_get_regs(BME280_DATA_ADDR, reg_data, BME280_P_T_H_DATA_LEN, dev);
        if (rslt == BME280_OK)
        {
            /* Parse the read data from the sensor */
            bme280_parse_sensor_data(reg_data, &uncomp_data);

            /* Compensate the pressure and/or temperature and/or
             * humidity data from the sensor
             */
            rslt = bme280_compensate_data(sensor_comp, &uncomp_data, comp_data, &dev->calib_data);
        }
    }
    else
    {
        rslt = BME280_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API is used to parse the pressure, temperature and
 *  humidity data and store it in the bme280_uncomp_data structure instance.
 */
void bme280_parse_sensor_data(const uint8_t *reg_data, struct bme280_uncomp_data *uncomp_data)
{
    /* Variables to store the sensor data */
    uint32_t data_xlsb;
    uint32_t data_lsb;
    uint32_t data_msb;

    /* Store the parsed register values for pressure data */
    data_msb = (uint32_t)reg_data[0] << 12;
    data_lsb = (uint32_t)reg_data[1] << 4;
    data_xlsb = (uint32_t)reg_data[2] >> 4;
    uncomp_data->pressure = data_msb | data_lsb | data_xlsb;

    /* Store the parsed register values for temperature data */
    data_msb = (uint32_t)reg_data[3] << 12;
    data_lsb = (uint32_t)reg_data[4] << 4;
    data_xlsb = (uint32_t)reg_data[5] >> 4;
    uncomp_data->temperature = data_msb | data_lsb | data_xlsb;

    /* Store the parsed register values for temperature data */
    data_lsb = (uint32_t)reg_data[6] << 8;
    data_msb = (uint32_t)reg_data[7];
    uncomp_data->humidity = data_msb | data_lsb;
}

/*!
 * @brief This API is used to compensate the pressure and/or
 * temperature and/or humidity data according to the component selected
 * by the user.
 */
int8_t bme280_compensate_data(uint8_t sensor_comp,
                              const struct bme280_uncomp_data *uncomp_data,
                              struct bme280_data *comp_data,
                              struct bme280_calib_data *calib_data)
{
    int8_t rslt = BME280_OK;

    if ((uncomp_data != NULL) && (comp_data != NULL) && (calib_data != NULL))
    {
        /* Initialize to zero */
        comp_data->temperature = 0;
        comp_data->pressure = 0;
        comp_data->humidity = 0;

        /* If pressure or temperature component is selected */
        if (sensor_comp & (BME280_PRESS | BME280_TEMP | BME280_HUM))
        {
            /* Compensate the temperature data */
            comp_data->temperature = compensate_temperature(uncomp_data, calib_data);
        }
        if (sensor_comp & BME280_PRESS)
        {
            /* Compensate the pressure data */
            comp_data->pressure = compensate_pressure(uncomp_data, calib_data);
        }
        if (sensor_comp & BME280_HUM)
        {
            /* Compensate the humidity data */
            comp_data->humidity = compensate_humidity(uncomp_data, calib_data);
        }
    }
    else
    {
        rslt = BME280_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This internal API sets the oversampling settings for pressure,
 * temperature and humidity in the sensor.
 */
static int8_t set_osr_settings(uint8_t desired_settings,
                               const struct bme280_settings *settings,
                               const struct bme280_dev *dev)
{
    int8_t rslt = BME280_W_INVALID_OSR_MACRO;

    if (desired_settings & BME280_OSR_HUM_SEL)
    {
        rslt = set_osr_humidity_settings(settings, dev);
    }
    if (desired_settings & (BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL))
    {
        rslt = set_osr_press_temp_settings(desired_settings, settings, dev);
    }

    return rslt;
}

/*!
 * @brief This API sets the humidity oversampling settings of the sensor.
 */
static int8_t set_osr_humidity_settings(const struct bme280_settings *settings, const struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t ctrl_hum;
    uint8_t ctrl_meas;
    uint8_t reg_addr = BME280_CTRL_HUM_ADDR;

    ctrl_hum = settings->osr_h & BME280_CTRL_HUM_MSK;

    /* Write the humidity control value in the register */
    rslt = bme280_set_regs(&reg_addr, &ctrl_hum, 1, dev);

    /* Humidity related changes will be only effective after a
     * write operation to ctrl_meas register
     */
    if (rslt == BME280_OK)
    {
        reg_addr = BME280_CTRL_MEAS_ADDR;
        rslt = bme280_get_regs(reg_addr, &ctrl_meas, 1, dev);
        if (rslt == BME280_OK)
        {
            rslt = bme280_set_regs(&reg_addr, &ctrl_meas, 1, dev);
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the pressure and/or temperature oversampling settings
 * in the sensor according to the settings selected by the user.
 */
static int8_t set_osr_press_temp_settings(uint8_t desired_settings,
                                          const struct bme280_settings *settings,
                                          const struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t reg_addr = BME280_CTRL_MEAS_ADDR;
    uint8_t reg_data;

    rslt = bme280_get_regs(reg_addr, &reg_data, 1, dev);
    if (rslt == BME280_OK)
    {
        if (desired_settings & BME280_OSR_PRESS_SEL)
        {
            fill_osr_press_settings(&reg_data, settings);
        }
        if (desired_settings & BME280_OSR_TEMP_SEL)
        {
            fill_osr_temp_settings(&reg_data, settings);
        }

        /* Write the oversampling settings in the register */
        rslt = bme280_set_regs(&reg_addr, &reg_data, 1, dev);
    }

    return rslt;
}

/*!
 * @brief This internal API sets the filter and/or standby duration settings
 * in the sensor according to the settings selected by the user.
 */
static int8_t set_filter_standby_settings(uint8_t desired_settings,
                                          const struct bme280_settings *settings,
                                          const struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t reg_addr = BME280_CONFIG_ADDR;
    uint8_t reg_data;

    rslt = bme280_get_regs(reg_addr, &reg_data, 1, dev);
    if (rslt == BME280_OK)
    {
        if (desired_settings & BME280_FILTER_SEL)
        {
            fill_filter_settings(&reg_data, settings);
        }
        if (desired_settings & BME280_STANDBY_SEL)
        {
            fill_standby_settings(&reg_data, settings);
        }

        /* Write the oversampling settings in the register */
        rslt = bme280_set_regs(&reg_addr, &reg_data, 1, dev);
    }

    return rslt;
}

/*!
 * @brief This internal API fills the filter settings provided by the user
 * in the data buffer so as to write in the sensor.
 */
static void fill_filter_settings(uint8_t *reg_data, const struct bme280_settings *settings)
{
    *reg_data = BME280_SET_BITS(*reg_data, BME280_FILTER, settings->filter);
}

/*!
 * @brief This internal API fills the standby duration settings provided by
 * the user in the data buffer so as to write in the sensor.
 */
static void fill_standby_settings(uint8_t *reg_data, const struct bme280_settings *settings)
{
    *reg_data = BME280_SET_BITS(*reg_data, BME280_STANDBY, settings->standby_time);
}

/*!
 * @brief This internal API fills the pressure oversampling settings provided by
 * the user in the data buffer so as to write in the sensor.
 */
static void fill_osr_press_settings(uint8_t *reg_data, const struct bme280_settings *settings)
{
    *reg_data = BME280_SET_BITS(*reg_data, BME280_CTRL_PRESS, settings->osr_p);
}

/*!
 * @brief This internal API fills the temperature oversampling settings
 * provided by the user in the data buffer so as to write in the sensor.
 */
static void fill_osr_temp_settings(uint8_t *reg_data, const struct bme280_settings *settings)
{
    *reg_data = BME280_SET_BITS(*reg_data, BME280_CTRL_TEMP, settings->osr_t);
}

/*!
 * @brief This internal API parse the oversampling(pressure, temperature
 * and humidity), filter and standby duration settings and store in the
 * device structure.
 */
static void parse_device_settings(const uint8_t *reg_data, struct bme280_settings *settings)
{
    settings->osr_h = BME280_GET_BITS_POS_0(reg_data[0], BME280_CTRL_HUM);
    settings->osr_p = BME280_GET_BITS(reg_data[2], BME280_CTRL_PRESS);
    settings->osr_t = BME280_GET_BITS(reg_data[2], BME280_CTRL_TEMP);
    settings->filter = BME280_GET_BITS(reg_data[3], BME280_FILTER);
    settings->standby_time = BME280_GET_BITS(reg_data[3], BME280_STANDBY);
}

/*!
 * @brief This internal API writes the power mode in the sensor.
 */
static int8_t write_power_mode(uint8_t sensor_mode, const struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t reg_addr = BME280_PWR_CTRL_ADDR;

    /* Variable to store the value read from power mode register */
    uint8_t sensor_mode_reg_val;

    /* Read the power mode register */
    rslt = bme280_get_regs(reg_addr, &sensor_mode_reg_val, 1, dev);

    /* Set the power mode */
    if (rslt == BME280_OK)
    {
        sensor_mode_reg_val = BME280_SET_BITS_POS_0(sensor_mode_reg_val, BME280_SENSOR_MODE, sensor_mode);

        /* Write the power mode in the register */
        rslt = bme280_set_regs(&reg_addr, &sensor_mode_reg_val, 1, dev);
    }

    return rslt;
}

/*!
 * @brief This internal API puts the device to sleep mode.
 */
static int8_t put_device_to_sleep(const struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data[4];
    struct bme280_settings settings;

    rslt = bme280_get_regs(BME280_CTRL_HUM_ADDR, reg_data, 4, dev);
    if (rslt == BME280_OK)
    {
        parse_device_settings(reg_data, &settings);
        rslt = bme280_soft_reset(dev);
        if (rslt == BME280_OK)
        {
            rslt = reload_device_settings(&settings, dev);
        }
    }

    return rslt;
}

/*!
 * @brief This internal API reloads the already existing device settings in
 * the sensor after soft reset.
 */
static int8_t reload_device_settings(const struct bme280_settings *settings, const struct bme280_dev *dev)
{
    int8_t rslt;

    rslt = set_osr_settings(BME280_ALL_SETTINGS_SEL, settings, dev);
    if (rslt == BME280_OK)
    {
        rslt = set_filter_standby_settings(BME280_ALL_SETTINGS_SEL, settings, dev);
    }

    return rslt;
}



/*!
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in integer data type.
 */
static int32_t compensate_temperature(const struct bme280_uncomp_data *uncomp_data,
                                      struct bme280_calib_data *calib_data)
{
    int32_t var1;
    int32_t var2;
    int32_t temperature;
    int32_t temperature_min = -4000;
    int32_t temperature_max = 8500;

    var1 = (int32_t)((uncomp_data->temperature / 8) - ((int32_t)calib_data->dig_T1 * 2));
    var1 = (var1 * ((int32_t)calib_data->dig_T2)) / 2048;
    var2 = (int32_t)((uncomp_data->temperature / 16) - ((int32_t)calib_data->dig_T1));
    var2 = (((var2 * var2) / 4096) * ((int32_t)calib_data->dig_T3)) / 16384;
    calib_data->t_fine = var1 + var2;
    temperature = (calib_data->t_fine * 5 + 128) / 256;
    if (temperature < temperature_min)
    {
        temperature = temperature_min;
    }
    else if (temperature > temperature_max)
    {
        temperature = temperature_max;
    }

    return temperature;
}

/*!
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in integer data type.
 */
static uint32_t compensate_pressure(const struct bme280_uncomp_data *uncomp_data,
                                    const struct bme280_calib_data *calib_data)
{
    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    uint32_t var5;
    uint32_t pressure;
    uint32_t pressure_min = 30000;
    uint32_t pressure_max = 110000;

    var1 = (((int32_t)calib_data->t_fine) / 2) - (int32_t)64000;
    var2 = (((var1 / 4) * (var1 / 4)) / 2048) * ((int32_t)calib_data->dig_P6);
    var2 = var2 + ((var1 * ((int32_t)calib_data->dig_P5)) * 2);
    var2 = (var2 / 4) + (((int32_t)calib_data->dig_P4) * 65536);
    var3 = (calib_data->dig_P3 * (((var1 / 4) * (var1 / 4)) / 8192)) / 8;
    var4 = (((int32_t)calib_data->dig_P2) * var1) / 2;
    var1 = (var3 + var4) / 262144;
    var1 = (((32768 + var1)) * ((int32_t)calib_data->dig_P1)) / 32768;

    /* avoid exception caused by division by zero */
    if (var1)
    {
        var5 = (uint32_t)((uint32_t)1048576) - uncomp_data->pressure;
        pressure = ((uint32_t)(var5 - (uint32_t)(var2 / 4096))) * 3125;
        if (pressure < 0x80000000)
        {
            pressure = (pressure << 1) / ((uint32_t)var1);
        }
        else
        {
            pressure = (pressure / (uint32_t)var1) * 2;
        }
        var1 = (((int32_t)calib_data->dig_P9) * ((int32_t)(((pressure / 8) * (pressure / 8)) / 8192))) / 4096;
        var2 = (((int32_t)(pressure / 4)) * ((int32_t)calib_data->dig_P8)) / 8192;
        pressure = (uint32_t)((int32_t)pressure + ((var1 + var2 + calib_data->dig_P7) / 16));
        if (pressure < pressure_min)
        {
            pressure = pressure_min;
        }
        else if (pressure > pressure_max)
        {
            pressure = pressure_max;
        }
    }
    else
    {
        pressure = pressure_min;
    }

    return pressure;
}


/*!
 * @brief This internal API is used to compensate the raw humidity data and
 * return the compensated humidity data in integer data type.
 */
static uint32_t compensate_humidity(const struct bme280_uncomp_data *uncomp_data,
                                    const struct bme280_calib_data *calib_data)
{
    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    int32_t var5;
    uint32_t humidity;
    uint32_t humidity_max = 102400;

    var1 = calib_data->t_fine - ((int32_t)76800);
    var2 = (int32_t)(uncomp_data->humidity * 16384);
    var3 = (int32_t)(((int32_t)calib_data->dig_H4) * 1048576);
    var4 = ((int32_t)calib_data->dig_H5) * var1;
    var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
    var2 = (var1 * ((int32_t)calib_data->dig_H6)) / 1024;
    var3 = (var1 * ((int32_t)calib_data->dig_H3)) / 2048;
    var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
    var2 = ((var4 * ((int32_t)calib_data->dig_H2)) + 8192) / 16384;
    var3 = var5 * var2;
    var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
    var5 = var3 - ((var4 * ((int32_t)calib_data->dig_H1)) / 16);
    var5 = (var5 < 0 ? 0 : var5);
    var5 = (var5 > 419430400 ? 419430400 : var5);
    humidity = (uint32_t)(var5 / 4096);
    if (humidity > humidity_max)
    {
        humidity = humidity_max;
    }

    return humidity;
}

/*!
 * @brief This internal API reads the calibration data from the sensor, parse
 * it and store in the device structure.
 */
static int8_t get_calib_data(struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t reg_addr = BME280_TEMP_PRESS_CALIB_DATA_ADDR;

    /* Array to store calibration data */
    uint8_t calib_data[BME280_TEMP_PRESS_CALIB_DATA_LEN] = { 0 };

    /* Read the calibration data from the sensor */
    rslt = bme280_get_regs(reg_addr, calib_data, BME280_TEMP_PRESS_CALIB_DATA_LEN, dev);
    if (rslt == BME280_OK)
    {
        /* Parse temperature and pressure calibration data and store
         * it in device structure
         */
        parse_temp_press_calib_data(calib_data, dev);
        reg_addr = BME280_HUMIDITY_CALIB_DATA_ADDR;

        /* Read the humidity calibration data from the sensor */
        rslt = bme280_get_regs(reg_addr, calib_data, BME280_HUMIDITY_CALIB_DATA_LEN, dev);
        if (rslt == BME280_OK)
        {
            /* Parse humidity calibration data and store it in
             * device structure
             */
            parse_humidity_calib_data(calib_data, dev);
        }
    }

    return rslt;
}

/*!
 * @brief This internal API interleaves the register address between the
 * register data buffer for burst write operation.
 */
static void interleave_reg_addr(const uint8_t *reg_addr, uint8_t *temp_buff, const uint8_t *reg_data, uint8_t len)
{
    uint8_t index;

    for (index = 1; index < len; index++)
    {
        temp_buff[(index * 2) - 1] = reg_addr[index];
        temp_buff[index * 2] = reg_data[index];
    }
}

/*!
 *  @brief This internal API is used to parse the temperature and
 *  pressure calibration data and store it in device structure.
 */
static void parse_temp_press_calib_data(const uint8_t *reg_data, struct bme280_dev *dev)
{
    struct bme280_calib_data *calib_data = &dev->calib_data;

    calib_data->dig_T1 = BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
    calib_data->dig_T2 = (int16_t)BME280_CONCAT_BYTES(reg_data[3], reg_data[2]);
    calib_data->dig_T3 = (int16_t)BME280_CONCAT_BYTES(reg_data[5], reg_data[4]);
    calib_data->dig_P1 = BME280_CONCAT_BYTES(reg_data[7], reg_data[6]);
    calib_data->dig_P2 = (int16_t)BME280_CONCAT_BYTES(reg_data[9], reg_data[8]);
    calib_data->dig_P3 = (int16_t)BME280_CONCAT_BYTES(reg_data[11], reg_data[10]);
    calib_data->dig_P4 = (int16_t)BME280_CONCAT_BYTES(reg_data[13], reg_data[12]);
    calib_data->dig_P5 = (int16_t)BME280_CONCAT_BYTES(reg_data[15], reg_data[14]);
    calib_data->dig_P6 = (int16_t)BME280_CONCAT_BYTES(reg_data[17], reg_data[16]);
    calib_data->dig_P7 = (int16_t)BME280_CONCAT_BYTES(reg_data[19], reg_data[18]);
    calib_data->dig_P8 = (int16_t)BME280_CONCAT_BYTES(reg_data[21], reg_data[20]);
    calib_data->dig_P9 = (int16_t)BME280_CONCAT_BYTES(reg_data[23], reg_data[22]);
    calib_data->dig_H1 = reg_data[25];
}

/*!
 *  @brief This internal API is used to parse the humidity calibration data
 *  and store it in device structure.
 */
static void parse_humidity_calib_data(const uint8_t *reg_data, struct bme280_dev *dev)
{
    struct bme280_calib_data *calib_data = &dev->calib_data;
    int16_t dig_H4_lsb;
    int16_t dig_H4_msb;
    int16_t dig_H5_lsb;
    int16_t dig_H5_msb;

    calib_data->dig_H2 = (int16_t)BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
    calib_data->dig_H3 = reg_data[2];
    dig_H4_msb = (int16_t)(int8_t)reg_data[3] * 16;
    dig_H4_lsb = (int16_t)(reg_data[4] & 0x0F);
    calib_data->dig_H4 = dig_H4_msb | dig_H4_lsb;
    dig_H5_msb = (int16_t)(int8_t)reg_data[5] * 16;
    dig_H5_lsb = (int16_t)(reg_data[4] >> 4);
    calib_data->dig_H5 = dig_H5_msb | dig_H5_lsb;
    calib_data->dig_H6 = (int8_t)reg_data[6];
}

/*!
 * @brief This internal API is used to identify the settings which the user
 * wants to modify in the sensor.
 */
static uint8_t are_settings_changed(uint8_t sub_settings, uint8_t desired_settings)
{
    uint8_t settings_changed = FALSE;

    if (sub_settings & desired_settings)
    {
        /* User wants to modify this particular settings */
        settings_changed = TRUE;
    }
    else
    {
        /* User don't want to modify this particular settings */
        settings_changed = FALSE;
    }

    return settings_changed;
}

/*!
 * @brief This internal API is used to validate the device structure pointer for
 * null conditions.
 */
static int8_t null_ptr_check(const struct bme280_dev *dev)
{
    int8_t rslt;

    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL))
    {
        /* Device structure pointer is not valid */
        rslt = BME280_E_NULL_PTR;
    }
    else
    {
        /* Device structure is fine */
        rslt = BME280_OK;
    }

    return rslt;
}
