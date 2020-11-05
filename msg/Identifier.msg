#Identificador de um objeto detectado

int32 TYPE_BASECOSTEIRA = 0 #Constante de tipo: base costeira
int32 TYPE_BASE = 1 #Constante de tipo: base
int32 TYPE_OLEODUTO = 2 #Constante de tipo: oleoduto
int32 TYPE_SENSOR_VERMELHO = 3 #Constante de tipo: sensor de monitoramento (fase 2)
int32 TYPE_SENSOR_VERDE = 4 #Constante de tipo: sensor de monitoramento (fase 2)
int32 TYPE_MOSTRADOR = 5 #Constante de tipo: mostrador digital (fase 3)
int32 TYPE_PACOTE_A = 6 #Constante de tipo: pacote (fase 4)
int32 TYPE_PACOTE_B = 7 #Constante de tipo: pacote (fase 4) 
int32 TYPE_PACOTE_C = 8 #Constante de tipo: pacote (fase 4)
int32 TYPE_PACOTE_D = 9 #Constante de tipo: pacote (fase 4) 
int32 TYPE_PACOTE_E = 10 #Constante de tipo: pacote (fase 4) 

int32 STATE_DESCONHECIDO = -1 #Constante de estado: desconhecido
int32 STATE_NOPROCESSADO = 0 #Constante de estado: não processado (ainda não foi realizada a ação que seria esperado)
int32 STATE_PROCESSADO = 1 #Constante de estado: processado (foi realizado a ação necessária)

std_msgs/Int32 type #Tipo do objeto
std_msgs/Int32 index #Índice do objeto (-1 para desconhecido)
std_msgs/Int32 state #Estado do objeto (-1 para desconhecido)
