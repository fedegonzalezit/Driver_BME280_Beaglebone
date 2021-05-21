#define PORT 10000


#define MAXIMOS_USUARIOS_DEFAULT 	1000
#define PERDIODO_MEDICION_DEFAULT	1
#define FILTRO_NUM_DEFAULT			5
#define BACKLOG						2


struct filtroMediaMovil
{
	uint32_t n;
	uint32_t *muestras;
	uint32_t filtroIndex;
};


union semun {
int              val;    /* Value for SETVAL */
struct semid_ds *buf;    /* Buffer for IPC_STAT, IPC_SET */
unsigned short  *array;  /* Array for GETALL, SETALL */
struct seminfo  *__buf;  /* Buffer for IPC_INFO
                           (Linux-specific) */
};

//Declaracion de funciones
void sigchld_handler(int sig);
void sigusr1_handler(int sig);
void atenderCliente (int new_socket, char * web);
void * thread_driver (void * arg);
int abrirWeb(char  ** web);
char * replaceHtml (char *web, char * hora, char * temperatura, char *humedad, char *presion);
void convertirResultado (uint32_t numero, int decimales, char * resultadoString);
void filtroDelete (struct filtroMediaMovil * Filtro);
int filtroAgregarDato (struct filtroMediaMovil *  Filtro, uint32_t dato);
int filtroPromediar (struct filtroMediaMovil * Filtro);
int setConfig (void);
int filtroInit (struct filtroMediaMovil * filtro);



