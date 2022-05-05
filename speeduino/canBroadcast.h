#ifndef CANBROADCAST_H
#define CANBROADCAST_H
#if defined(NATIVE_CAN_AVAILABLE)

//For BMW e46/e39/e38, rover and mini other CAN instrument clusters
#define CAN_BMW_ASC1 0x153 //Rx message from ACS unit that includes speed
#define CAN_BMW_DME1 0x316 //Tx message that includes RPM
#define CAN_BMW_DME2 0x329 //Tx message that includes CLT and TPS
#define CAN_BMW_DME4 0x545 //Tx message that includes CLT and TPS
#define CAN_BMW_ICL2 0x613
#define CAN_BMW_ICL3 0x615

//For VAG CAN instrument clusters
#define CAN_VAG_RPM 0x280
#define CAN_VAG_VSS 0x5A0

void sendBMWCluster();
void sendVAGCluster();
void sendHaltech30Hz(); //50Hz rate in Haltech, but 30Hz for now in speeduino
void sendHaltech15Hz(); //20Hz rate in Haltech, but 15Hz for now in speeduino
void sendHaltech10Hz(); //10Hz rate in Haltech
void sendHaltech4Hz(); //5Hz rate in Haltech, but 4Hz for now in speeduino

void dashMessage(uint16_t dashMessageID);
void haltechDashMessage(uint16_t dashMessageID);
#endif
#endif // CANBROADCAST_H
