// Run command line: gcc CNPW.c -o CPNW -lm -lpthread

#include <unistd.h>
#include <stdio.h>
#include <time.h>
#include <errno.h>
#include <fcntl.h> 
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <signal.h>
#include <limits.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <pthread.h>
#include <net/if.h>
#include <pthread.h>
#include <math.h>
#define SIZE_BUFFER 8
 
char buffer_tx[] = {0,0,0,0,0,0,0,0};
char buffer_rx[] = {0,0,0,0,0,0,0,0};
 
time_t timer;
struct tm *horarioLocal;
int dia, mes, ano, hora, min, sec;
 
volatile unsigned short int msg_receive = 0, nbytes = 0;
volatile unsigned short int running = 1;
unsigned short int nwrite;
unsigned short int nread;
pthread_t thread_id;
char *portname = "/dev/ttyACM0";
int handle; 
int n = SIZE_BUFFER;
unsigned short int anomalias = 0;
void signal_Handler(int sign){
    int o_exit = 0;
    switch (sign){
        case SIGINT:{
            o_exit = 1;
            break;
        }
        case SIGTERM:{
            o_exit = 1;
            break;
        }
        default:{
            break;
        }
    }
 
    if (o_exit == 1)
    running = 0;
}
 
void *uart_msg_receive(void *t){
    while(running){
        for(n = 0; n < SIZE_BUFFER; n++){
            nread = read(handle,&buffer_rx[n],1);
            /*if(nread < 0){
                printf("Read error\n");
            }else{
                printf("Data (%d) read = %d \n",n,buffer_rx[n]);
            }*/     
        }       
        msg_receive = 1;
        usleep(1000);
        while(msg_receive);
    }
}
 
int set_interface_attribs(int fd, int speed){
    struct termios tty;
    if (tcgetattr(fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }
    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);
    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;                 /* 8-bit characters */
    tty.c_cflag &= ~PARENB;             /* no parity bit */
    tty.c_cflag &= ~CSTOPB;             /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;            /* no hardware flowcontrol */
    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;
    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}
 
void set_mincount(int fd, int mcount){
    struct termios tty;
    if (tcgetattr(fd, &tty) < 0) {
        printf("Error tcgetattr: %s\n", strerror(errno));
        return;
    }
    tty.c_cc[VMIN] = mcount ? 1 : 0;
    tty.c_cc[VTIME] = 5;        /* half second timer */
    if (tcsetattr(fd, TCSANOW, &tty) < 0)
        printf("Error tcsetattr: %s\n", strerror(errno));
}
 
void update_time(void){
    time(&timer);
    horarioLocal = localtime(&timer);
    dia = horarioLocal->tm_mday;
    mes = horarioLocal->tm_mon + 1;
    ano = horarioLocal->tm_year + 1900;
    hora = horarioLocal->tm_hour;
    min  = horarioLocal->tm_min;
    sec  = horarioLocal->tm_sec;
}
 
 
int main(void){ 
 
    /////
    unsigned int i = 0, count = 0;
    unsigned char c = 0, wave_complete = 0, harm_ord = 1;
    float v_vector[2048], f = 0, v = 0, p = 0, q =0;
    float Mag_vg[53], Mag_ig[53]; 
    /////
    char query[512], str[30];
    FILE *dataloger;
    FILE *temperatureFile;
 
    handle = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (handle < 0) {
        printf("Error opening %s: %s\n", portname, strerror(errno));
        return -1;
    }    
    set_interface_attribs(handle, B115200);// baudrate 115200, 8 bits, no parity, 1 stop bit
    
    if( pthread_create(&thread_id, NULL, &uart_msg_receive, NULL) ){
        printf("\nErro thread\n");
    }
    
    for(i = 0; i < 2048; i++){
        v_vector[i] = 0.0;
    }
    
    signal(SIGINT,  signal_Handler);
    signal(SIGTERM, signal_Handler);
    time(NULL);
 
    sleep(1);
    
    buffer_tx[0] = 0xF0;
    buffer_tx[1] = 0x01;
    nwrite = write(handle,buffer_tx,8);
    tcdrain(handle);
 
    while (running){
        if(msg_receive){
            if( (buffer_rx[0] >> 4) == 1 ){
                
                i = ((buffer_rx[0] & 0x07) << 8) + (buffer_rx[1] & 0xFF);
                for(c = 2; c < 8; c++){
                    if(i < 2048){
                        if( (buffer_rx[c] & 0x80) == 0x80)
                            v_vector[i] = -0.01*( (float)((0xFF ^ buffer_rx[c]) + 1) );
                        else
                            v_vector[i] = 0.01*((float)buffer_rx[c]);
                        i++;                        
                    }               
                }
                
                if(i == 2048){
                    anomalias++;
                    wave_complete = 1;
                }
            }else if( (buffer_rx[0] >> 4) == 2 ){
                if( (buffer_rx[2] & 0x80) == 0x80)
                    v = -0.1*( (float)((( 0xFF ^ buffer_rx[2])<<8) + ( 0xFF ^ buffer_rx[3]) + 1 ) );
                else
                    v = 0.1*( (float)((buffer_rx[2]<<8) + (buffer_rx[3]) ) );
 
                if( (buffer_rx[4] & 0x80) == 0x80)
                    f = -0.01*( (float)((( 0xFF ^ buffer_rx[4])<<8) + ( 0xFF ^ buffer_rx[5]) + 1 ) );
                else
                    f = 0.01*( (float)((buffer_rx[4]<<8) + (buffer_rx[5]) ) );
 
                if( (buffer_rx[6] & 0x80) == 0x80)
                    p = -0.01*( (float)( (0xFF ^ buffer_rx[6]) + 1) );
                else
                    p = 0.01*( (float)((int)buffer_rx[6]));
 
                if( (buffer_rx[7] & 0x80) == 0x80)
                    q = -0.01*( (float)( (0xFF ^ buffer_rx[7]) + 1) );
                else
                    q = 0.01*( (float)buffer_rx[7]);
                
                update_time();
 
                dataloger = fopen("dataloger_1.csv","a");
                if(dataloger == NULL){
                    printf("\nErro dataloger\n!");
                    exit(1);
                }           
                sprintf(query,"%d,%d,%d,%d,%d,%d,%.2f,%.2f,%.2f,%.2f\n",dia,mes,ano,hora,min,sec,v,f,p,q);
                fprintf(dataloger,"%s",query);          
                
                fclose(dataloger);              
            }else if( (buffer_rx[0] >> 4) == 3 ){
                
                for(c = 1; c < 8; c++){                 
                    Mag_vg[harm_ord] = 0.1*((float)((unsigned)buffer_rx[c]));
                    harm_ord = harm_ord+2;
                    if(harm_ord == 53){
                        sprintf(query,"v,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",Mag_vg[1],Mag_vg[3],Mag_vg[5],Mag_vg[7],Mag_vg[9],Mag_vg[11],Mag_vg[13],Mag_vg[15],Mag_vg[17],Mag_vg[19],Mag_vg[21],Mag_vg[23],Mag_vg[25],Mag_vg[27],Mag_vg[29],Mag_vg[31],Mag_vg[33],Mag_vg[35],Mag_vg[37],Mag_vg[39],Mag_vg[41],Mag_vg[43],Mag_vg[45],Mag_vg[47],Mag_vg[49],Mag_vg[51]);
                        dataloger = fopen("dataloger_2.csv","a");
                        if(dataloger == NULL){
                            printf("\nErro fft dataloger\n!");
                            exit(1);
                        }else{      
                            fprintf(dataloger,"%s",query);              
                        }
                        harm_ord = 1;
                        fclose(dataloger);
                        break;
                    }                   
                }
                
            }else if( (buffer_rx[0] >> 4) == 4 ){
                
                for(c = 1; c < 8; c++){                 
                    Mag_ig[harm_ord] = 0.1*((float) ((unsigned)buffer_rx[c]) );
                    harm_ord = harm_ord+2;
                    if(harm_ord == 53){
                        sprintf(query,"i,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",Mag_ig[1],Mag_ig[3],Mag_ig[5],Mag_ig[7],Mag_ig[9],Mag_ig[11],Mag_ig[13],Mag_ig[15],Mag_ig[17],Mag_ig[19],Mag_ig[21],Mag_ig[23],Mag_ig[25],Mag_ig[27],Mag_ig[29],Mag_ig[31],Mag_ig[33],Mag_ig[35],Mag_ig[37],Mag_ig[39],Mag_ig[41],Mag_ig[43],Mag_ig[45],Mag_ig[47],Mag_ig[49],Mag_ig[51]);
                        dataloger = fopen("dataloger_2.csv","a");
                        if(dataloger == NULL){
                            printf("\nErro fft dataloger\n!");
                            exit(1);
                        }else{      
                            fprintf(dataloger,"%s",query);              
                        }
                        harm_ord = 1;
                        fclose(dataloger);
                        break;
                    }                   
                }           
            }
 
            memset(buffer_rx,0,sizeof(buffer_rx));
            msg_receive = 0;
        }
        
        if(wave_complete){
            
            update_time();
            
            dataloger = fopen("dataloger_0.csv","a");
            if(dataloger == NULL){
                printf("\nErro dataloger\n!");
                exit(1);
            }           
            for(i = 0; i < 2048; i++){              
                sprintf(query,"%d,%d,%d,%d,%d,%d,%d,%.3f\n",dia,mes,ano,hora,min,sec,i,v_vector[i]);
                fprintf(dataloger,"%s",query);          
            }
            fclose(dataloger);
            memset(buffer_tx,0,sizeof(buffer_rx));
            buffer_tx[0] = 0xF1;
            nwrite = write(handle,buffer_tx,8);
            tcdrain(handle);
            for(i = 0; i < 2048; i++){
                v_vector[i] = 0.0;
            }
            wave_complete = 0;
        }
        
        usleep(2000);
        count++;
        if(count == 200){
            count = 0;
            update_time();
        }
    }
    buffer_tx[0] = 0xF0;
    buffer_tx[1] = 0x00;
    nwrite = write(handle,buffer_tx,8);
    tcdrain(handle);
 
    close(handle);
    printf("\nStop program\n");
    return 0;   
}
