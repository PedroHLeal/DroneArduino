#include <AltSoftSerial.h>

class BluetoothSingleton {
  public:
    static AltSoftSerial *b;
    static AltSoftSerial* getBluetooth() {
      if (b == NULL) {
        b = new AltSoftSerial();
        b->begin(9600);
      }
      return b;
    }
};

AltSoftSerial* BluetoothSingleton::b = NULL;
