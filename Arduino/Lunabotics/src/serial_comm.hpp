#include <stdlib.h>
#include <stdbool.h>

struct {
    int rate; //baudrate of serial connection to jetson
    bool reliable; //is it using a TCP or UDP type of connection
    bool connected; //has this connection been connected and verified
    double lastMsgTime; //time stamp of the last msg
    double startConTime; //time stamp of when the connection started

} connection;

enum msgType {
    motor,
    encoder,
    other
};

class messagePacket{
    public:
        msgType type;
        int size; //size of the data array (indicies)
        double timeStamp; //time stamp of message creation
        double* data; //data of the message
        bool init(int size);        
        
};

bool sendData(double* data);
bool getData();