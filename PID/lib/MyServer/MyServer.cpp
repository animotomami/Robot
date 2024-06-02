#include "MyServer.h"

WiFiServer _server(55355); // ¡¡¡ CAMBIAR POR NUM DE PUERTO DESEADO !!!
std::vector<struct MyClient> _conn;

bool CheckNewConn() {
    WiFiClient new_conn=_server.available();
    if (new_conn) { // nuevo cliente ha solicitado conexión
        struct MyClient remote;
        remote.cli=new_conn;
        remote.name="UNKNOWN";
        int j=_conn.size();
        _conn.push_back(remote); // añade a lista
        IPAddress remoteIp=remote.cli.remoteIP();
        printf("Cliente %d conectado. Remote IP: %d,%d.%d.%d , port:%d\n",
        j, remoteIp[0],remoteIp[1],remoteIp[2],remoteIp[3],
        remote.cli.remotePort());
        // Solicita identificación al cliente
        char ans[]="<<<NAME>>>";
        printf("Enviando: %s",ans);
        _conn[j].cli.write(ans);
        return true;
    }
    return false;
}

int CheckReceive() {
    for (int j=0;j<_conn.size();j++) { // para todos los clientes
        if (_conn[j].cli.available() ) { // se ha recibido algo del cliente
            _conn[j].recv=_conn[j].cli.readStringUntil('\n'); // actualiza recv
            //printf("Recibido cliente %d: %s\n",j,_conn[j].recv.c_str());
            char ans[64];
            if (_conn[j].recv.startsWith("<<<NAME=")) { // es identificación
            int end=_conn[j].recv.indexOf(">>>");
            _conn[j].name=_conn[j].recv.substring(8,end); // actualiza name
            _conn[j].recv="";
            }
            else if (_conn[j].name=="UNKNOWN") { // aún no identificado
            strcpy(ans,"<<<NAME>>>\n"); // repite solicitud
            printf("Enviando: %s",ans);
            _conn[j].cli.write(ans);
            }
            else {
            return j; // Devuelve el nº del cliente
            }
        }
        else if (! _conn[j].cli.connected() ) { // Cliente desconectado
            printf("Cliente %d (%s) desconectado\n",j,_conn[j].name.c_str() );
            _conn.erase(_conn.begin()+j); // Elimina de la lista
        }
    }
    return -1;
}

int SendTo(int j,const char* txt) {
    return _conn[j].cli.write(txt);
}

int SendTo(const char* name,const char* txt) {
    for (int j=0;j<_conn.size();j++) {
        if (_conn[j].name==name) // El nombre concide
        return SendTo(j,txt);
    }
    return -1;
}

int SendToAllStartingWith(const char* name,const char* txt) {
    int count=0;
    for (int j=0;j<_conn.size();j++) {
    if (_conn[j].name.startsWith(name)) {// El nombre concide
        SendTo(j,txt);
        count++;
    }
    }
    return count;
}