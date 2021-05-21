#include <unistd.h> 
#include <stdio.h> 
#include <sys/socket.h> 
#include <stdlib.h> 
#include <netinet/in.h> 
#include <string.h> 
#include <signal.h>
#include <string.h>
#include <pthread.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <wait.h>
#include <math.h>
#include "../inc/servidor.h"
#include <semaphore.h>
#include <time.h>
#include <fcntl.h>
#include <sys/ipc.h>
#include <sys/sem.h>


//Variables globales y variables de configuración
int contadorUsuarios = 0;
int usuariosMaximos = 0;
int periodoMaximo;
int numFiltro;
char *shared_memory_temp = NULL;
char *shared_memory_press = NULL;
char *shared_memory_hum = NULL;
char *shared_memory_hora = NULL;

int FlagConfig = 0;
int PadrePID;


int main(int argc, char const *argv[]) 
{ 
    int server_fd, new_socket, p; 
    struct sockaddr_in address; 
    int opt = 1; 
    int response;
    int addrlen = sizeof(address);
    char *web = NULL;
    int err;
    pthread_t thread_id;
    key_t key;
    int semid;
    union semun arg;


    if ((key = ftok("servidor.c", 'J')) == -1) {
        perror("ftok");
        exit(1);
    }

    /* create a semaphore set with 1 semaphore: */
    if ((semid = semget(key, 1, 0666 | IPC_CREAT)) == -1) {
        perror("semget");
        exit(1);
    }

    /* initialize semaphore #0 to 1: */
    arg.val = 1;
    if (semctl(semid, 0, SETVAL, arg) == -1) {
        perror("semctl");
        exit(1);
    }




    
    //Seteo configuracion
    err = setConfig();
    if (err < 0)
    {
        printf("Error al configurar, se setea default\n");
        usuariosMaximos = MAXIMOS_USUARIOS_DEFAULT;
        periodoMaximo = PERDIODO_MEDICION_DEFAULT;
        numFiltro = FILTRO_NUM_DEFAULT;
    }


    //Handler de señales
    signal (SIGCHLD, &sigchld_handler);
    signal (SIGUSR1, &sigusr1_handler);

    PadrePID = getpid();
    //Crea el thread que va a usar el driver
    if(pthread_create (&thread_id, NULL, &thread_driver, NULL) != 0)
    {
        perror("Error al crear hilo");
        exit(EXIT_FAILURE);
    }


    err = abrirWeb(&web);
    if (err < 0)
    {
        perror("Fallo al abrir la pagina web");
        exit(EXIT_FAILURE);
    }


    // Creating socket file descriptor 
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) 
    { 
        perror("socket failed"); 
        exit(EXIT_FAILURE); 
    } 

    // Forcefully attaching socket to the port 8080 
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, 
                                                  &opt, sizeof(opt))) 
    { 
        perror("setsockopt"); 
        exit(EXIT_FAILURE); 
    } 

    address.sin_family = AF_INET; 
    address.sin_addr.s_addr = INADDR_ANY; 
    address.sin_port = htons( PORT ); 

    // Forcefully attaching socket to the port 8080 
    if (bind(server_fd, (struct sockaddr *)&address,  
                                 sizeof(address))<0) 
    { 
        perror("bind failed"); 
        exit(EXIT_FAILURE); 
    } 


    if (listen(server_fd, BACKLOG) < 0) 
    { 
        perror("listen failed"); 
        exit(EXIT_FAILURE); 
    } 

    while (1)
    {
        if ((new_socket = accept(server_fd, (struct sockaddr *)&address,  
            (socklen_t*)&addrlen))<0) 
        { 
            perror("accept"); 
            exit(EXIT_FAILURE); 
       }
        printf("Acepta conexion\n");

        if (contadorUsuarios < usuariosMaximos)
        {
            contadorUsuarios++;
            p = fork ();
            if (p == 0)
            {
                    printf("Crea hijo\n");

                    close (server_fd);
                    atenderCliente (new_socket, web);

                    // shutdown (new_socket);
                    close (new_socket);
                    exit(1);
                    //return 0;
            }
            else
            {

                    close (new_socket);

            }
        }

    }

    free (web);
    free (shared_memory_temp);
    free (shared_memory_press);
    free (shared_memory_hum);
    
    /* remove it: */
    if (semctl(semid, 0, IPC_RMID, arg) == -1) {
        perror("semctl");
        exit(1);
    }


} 


/*
* Thread para leer driver
* La funcion read devuelve los 4 primeros bytes la temperatura y los 4 segundos la presion
*/
void * thread_driver (void * arg)
{
    int fd;
    int err;
    char buffer[12];
    uint32_t temp;
    int press;
    int hum;
    struct filtroMediaMovil FiltroTemperatura;
    struct filtroMediaMovil FiltroPresion;
    struct filtroMediaMovil FiltroHumedad;
    char temp_string[5];  //(2 enteros, 2 decimales y una coma, ver resolucion de bmp)
    char press_string[7];
    char hum_string [5];
    char hora_string[32];
    time_t T;       //Estructura para tomar la fecha de la última medición
    struct tm *timeNow; // ^
    int semid;
    struct sembuf sb = {0, -1, 0};  /* set to allocate resource */
    key_t key;
    int restante = 0;


    //Creo el semaforo que se va a usar para comunicar el thread del proceso padre con el proceso hijo cliente
    if ((key = ftok("servidor.c", 'J')) == -1) {
        perror("ftok");
        exit(1);
    }


    //Inicializacion del filtros, uno para cada variable a medir
    filtroInit(&FiltroTemperatura);
    filtroInit(&FiltroPresion);
    filtroInit(&FiltroHumedad);


    //Inicializo memoria compartida

    shared_memory_temp = mmap(NULL, 32, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    shared_memory_press = mmap(NULL, 32, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    shared_memory_hum = mmap(NULL, 32, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    shared_memory_hora = mmap(NULL, 32, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);


    /*Creación de la memoria compartida */



    fd = open ("/dev/spi-td3", O_RDWR);
    if (fd < 0)
    {
        printf("error al abrir el driver\n");
        return 0;
    }
    else
    {

        while (1)
        {

            if (restante == 0)
            {
                restante = periodoMaximo;
                if (1 == FlagConfig)
                {
                    //Si se cambio la configuración se tienen que resetear los filtros, porque se pudo haber cambiado N
                    filtroDelete (&FiltroTemperatura);
                    filtroDelete (&FiltroPresion);
                    filtroDelete (&FiltroHumedad);

                    filtroInit(&FiltroTemperatura);
                    filtroInit(&FiltroPresion);
                    filtroInit(&FiltroHumedad);            

                    FlagConfig = 0;
                }
                
                err = read (fd, buffer, 12);
                if (err < 0)
                {
                    printf("error al leer datos\n");
                }
                else
                {
                    temp = 0;
                    temp = (buffer[3]<<24) | (buffer[2]<<16) | (buffer[1]<<8) | buffer[0];
                    press = 0;
                    press = (buffer[7]<<24) | (buffer[6]<<16) | (buffer[5]<<8) | buffer[4];
                    hum = 0;
                    hum = (buffer[11]<<24) | (buffer[10]<<16) | (buffer[9]<<8) | buffer[8];


                    filtroAgregarDato(&FiltroTemperatura, temp);
                    temp = filtroPromediar(&FiltroTemperatura);
                    convertirResultado(temp, 2, temp_string);       //Resultado sin coma, lo convierte a string con coma

                    filtroAgregarDato(&FiltroPresion, press);
                    press = filtroPromediar(&FiltroPresion);
                    convertirResultado(press, 2, press_string);       //Resultado sin coma, lo convierte a string con coma

                    filtroAgregarDato(&FiltroHumedad, hum);
                    hum = filtroPromediar(&FiltroHumedad);
                    convertirResultado(hum, 3, hum_string);       //Resultado sin coma, lo convierte a string con coma


                    time(&T);
                    timeNow = localtime(&T); 
                    sprintf(hora_string, "%02d:%02d:%02d", timeNow->tm_hour, timeNow->tm_min, timeNow->tm_sec);

                    printf("PID: %d \n", getpid);
                    printf("Hora: %s \n", hora_string);
                    printf("Humedad: %s\n", hum_string);

                    /* grab the semaphore set created by seminit.c: */
                    if ((semid = semget(key, 1, 0)) == -1) {
                        perror("semget");
                        exit(1);
                    }

                    strcpy (shared_memory_temp, temp_string);
                    strcpy (shared_memory_hum, hum_string);
                    strcpy (shared_memory_press, press_string);
                    strcpy (shared_memory_hora, hora_string);

                    sb.sem_op = 1; /* free resource */
                    if (semop(semid, &sb, 1) == -1) {
                        perror("semop");
                        exit(1);
                    }
                } 


            }
  
		

            restante = sleep (restante);
        }
    }

   
    
    close (fd);
    
    filtroDelete(&FiltroTemperatura);
    filtroDelete(&FiltroHumedad);
    filtroDelete(&FiltroPresion);

}


/*
Funciones del cliente
*/
void atenderCliente (int new_socket, char * web)
{
    char httpHeader[] = "HTTP/1.1 200 OK connection:keep-alive\ncontent-type:text/html;charset=UTF-8\ncontent-length:%l\n\n";
    int valread;
    char buffer[1024] = {0};
    char *pagina = NULL;
    valread = read( new_socket , buffer, 1024);
    char http_buf[15000];
    char * cmp = NULL;
    int semid;
    struct sembuf sb = {0, -1, 0};  /* set to allocate resource */
    key_t key;



    //Creo el semaforo que se va a usar para comunicar el thread del proceso padre con el proceso hijo cliente
    if ((key = ftok("servidor.c", 'J')) == -1) {
        perror("ftok");
        exit(1);
    }


    //Se fija si el cliente pidio una conexión http
    cmp = strstr(buffer, "GET / HTTP/1.1");
    //printf("buffer: %s\n", buffer);


    //Poner un semaforo al tomar datos de la memoria compartida
    /* grab the semaphore set created: */
    if ((semid = semget(key, 1, 0)) == -1) {
        perror("semget");
        exit(1);
    }

    pagina = replaceHtml (web, shared_memory_hora, shared_memory_temp, shared_memory_hum, shared_memory_press);

    sb.sem_op = 1; /* free resource */
    if (semop(semid, &sb, 1) == -1) {
        perror("semop");
        exit(1);
    }
    //Termino de tomar datos, saco el semaforo

    memcpy(http_buf, httpHeader, strlen(httpHeader));
    strcat(http_buf, pagina);


    if (cmp != NULL)
    {
        if (send(new_socket , http_buf , strlen(http_buf) , 0 ) < 0) 
        {
            perror("Error al atender cliente");
            exit(EXIT_FAILURE);
        }

    }


    free(pagina);
}
    
/*
* Funcion que abre archivo que contiene HTML para ser enviado por el servidor al cliente
*/
int abrirWeb(char  ** web)
{
    FILE * f;
    int size;
    char * line;
    f = fopen ("../html/index.html", "r"); //Abre archivo html con la web
    if (f == NULL)
    {
        printf("No se pudo abrir html\n");
        return -1;
    }
    fseek (f, 0, SEEK_END); //va al final del archivo
    size = ftell (f); //Devuelve la posicion

    *web = (char *)malloc(size * sizeof(char));
    line = (char *)malloc(size * sizeof(char));
    fseek (f, 0, SEEK_SET); //vuelve al principio

    while (fgets (line, size, f) != NULL)
    {
        strncat(*web, line, strlen(line));
    }

    free (line);

    fclose (f);
    return 0;
}

/*
*   Funcion que recibe el template de la web y te agrega los parametros medidos
*   Devuelve un nuevo string con los parametros cambiados
*/


char * replaceHtml (char *web, char * hora, char * temperatura, char *humedad, char *presion)
{

    char * dest = malloc (strlen(web)+strlen(hora)-2+ strlen(temperatura)-2+ strlen(humedad)-2+strlen(presion)-2+
        strlen("Federico Gonzalez Itzik")-2 + strlen("157067-7")-2);   //agrega el largo de cada palabra y le saca el %s (2)
    

    sprintf (dest,web,"Federico Gonzalez Itzik", "157067-7", hora, temperatura, presion, humedad);
    
    return dest;
}


/*
Interrupciones
*/
void sigchld_handler (int sig)
{
        while (wait(NULL) > 0); //Limpia todos los hijos
        contadorUsuarios--;
        printf("Contador de usuarios: %d\n", contadorUsuarios);
}


void sigusr1_handler (int sig)
{
    int err;

    printf("Señal USR1: pid: %d\n", getpid());

    FlagConfig = 1;
    err = setConfig();
    if (err < 0)
    {
        printf("Error al configurar, se setea default\n");
        usuariosMaximos = MAXIMOS_USUARIOS_DEFAULT;
        periodoMaximo = PERDIODO_MEDICION_DEFAULT;
        numFiltro = FILTRO_NUM_DEFAULT;
    }



}

int setConfig (void)
{

    FILE *fconf;
    char buffer[25];
    int aux;

    fconf = fopen ("conf.txt", "r");
    if (fconf == NULL)
    {
        perror("Conf failed");
        return -1;
    }

    //En la primera posición esta el numero máximo de usuarios
    fgets(buffer, 25, fconf);
    aux = atoi(buffer);
    if (aux > 0)
    {
        usuariosMaximos = aux;
        printf("Maximos usuarios: %d\n", usuariosMaximos);
    }
    else
    {
        printf("%s%d\n", "Configuracón de maximos usuarios incorrecta, se leyo: ", aux);
        return -1;
    }

    //En la segunda posición está el periodo de medición
    fgets(buffer, 25, fconf);
    aux = atoi(buffer);

    if (aux > 0)
    {
        periodoMaximo = aux;
        printf("Periodo medicion: %d\n", periodoMaximo);
    } 
    else
    {
        printf("%s%d\n", "Configuracón de periodo incorrecta, se leyo: ", aux);
        return -1;

    }

    //En la tercer linea está el filtro de media movil
    fgets(buffer, 25, fconf);
    aux = atoi(buffer);

    if (aux > 0)
    {
        numFiltro = aux;
        printf("N filtro: %d\n", numFiltro);
    } 
    else
    {
        printf("%s%d\n", "Configuracón de filtro incorrecta, se leyo: ", aux);
        return -1;

    }

    fclose(fconf);

}

/*
Funciones para Filtro de media movil
*/

int filtroInit (struct filtroMediaMovil * filtro)
{

    filtro->n = numFiltro;
    filtro->muestras = (uint32_t *) malloc (filtro->n * sizeof(uint32_t));
    filtro->filtroIndex = 0;

    if (filtro->muestras == NULL)
    {
        printf("Error al inicializar el filtro\n");
        return -1; 
    }
    
    return 0;

}

void filtroDelete (struct filtroMediaMovil * Filtro)
{
    free (Filtro->muestras);
    Filtro->filtroIndex = 0;
}

int filtroAgregarDato (struct filtroMediaMovil * Filtro, uint32_t dato)
{
    Filtro->muestras[Filtro->filtroIndex] = dato;
    Filtro->filtroIndex++;
    Filtro->filtroIndex %= Filtro->n;
}

int filtroPromediar (struct filtroMediaMovil * Filtro)
{
    int i;
    int result = 0;

    for (i=0; i<Filtro->n; i++)
    {
        result += Filtro->muestras[i];
    }

    result /= Filtro->n;
    return result;
}

/*
*   Funcion para convertir entero string con coma
*   Debe recibir una string vacia
*/

void convertirResultado (uint32_t numero, int decimales, char * resultadoString)
{
    int entero;
    int decimal;
    int div;

    div = pow(10,decimales);

    entero = numero/div;
    decimal = numero % div;

    sprintf (resultadoString,"%d,%d",entero, decimal);


}

