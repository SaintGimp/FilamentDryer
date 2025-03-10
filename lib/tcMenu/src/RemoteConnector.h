/*
 * Copyright (c) 2018 https://www.thecoderscorner.com (Nutricherry LTD).
 * This product is licensed under an Apache license, see the LICENSE file in the top-level directory.
 */

/**
 * @file RemoteConnector.h
 * Contains the base functionality for communication between the menu library and remote APIs.
 */

#ifndef _TCMENU_REMOTECONNECTOR_H_
#define _TCMENU_REMOTECONNECTOR_H_

#include <Arduino.h>
#include "RemoteTypes.h"
#include <tcUtil.h>
#include "MenuItems.h"
#include "RuntimeMenuItem.h"
#include "MessageProcessors.h"
#include "MenuIterator.h"

#define TAG_VAL_PROTOCOL 0x01
#define START_OF_MESSAGE 0x01
#define TICK_INTERVAL 20
#define HEARTBEAT_INTERVAL 2500
// when debugging you can increase the heartbeat time to reduce disconnects (below gives 90 seconds)
//#define HEARTBEAT_INTERVAL 30000
#define HEARTBEAT_INTERVAL_TICKS (HEARTBEAT_INTERVAL / TICK_INTERVAL)
#define PAIRING_TIMEOUT_TICKS (15000 / TICK_INTERVAL)

/**
 * @file RemoteConnector.h
 * 
 * This class contains the majority of the code for dealing with remote connections using the TagVal protocol.
 */

/**
 * This enum describes the various states that a field and value object can be in. Field and value is
 * basically a simple state machine that remembers where the incoming communication was up to last time
 * around, so it can be processed asynchronously.
 * @see FieldAndValue
 */
enum FieldValueType : byte {
    /** a new message has arrived */
	FVAL_NEW_MSG,
    /** the end of the present message has been located */
    FVAL_END_MSG, 
    /** a new field on the message has been found */
    FVAL_FIELD, 
    /** there has been an error while reading the message */
    FVAL_ERROR_PROTO,
	/** waiting for a field key */
	FVAL_PROCESSING, 
    /** waiting for the = sign following the key */
    FVAL_PROCESSING_WAITEQ, 
    /** waiting for the value after finding the equals sign */
    FVAL_PROCESSING_VALUE, 
    /** waiting for a new message */
    FVAL_PROCESSING_AWAITINGMSG, 
    /** waiting to find the protocol type */
    FVAL_PROCESSING_PROTOCOL,
    /** waiting to find the first message type in the header */
    FVAL_PROCESSING_MSGTYPE_HI,
    /** waiting to find the second message type in the header */
    FVAL_PROCESSING_MSGTYPE_LO
};

/** 
 * This class describes the ongoing processing of an incoming message. In the embedded domain where we
 * are essentially single threaded and even on 32 bit fairly memory constrained, there needs to be a
 * minimalist way to process incoming events. This class processes data asynchronously by reading in
 * a byte at a time and slowly updating it's state. It has many states, but more generally the states
 * containing the word PROCESSING mean that there is nothing yet ready for use, these will never be
 * passed externally to a message processor.
 */
struct FieldAndValue {
	FieldValueType fieldType;
	uint16_t msgType;
	uint16_t field;
	char value[MAX_VALUE_LEN];
	uint8_t len;
};

/**
 * This is used by the notification callback to indicate the latest state of a connection.
 */

#define COMMSERR_OK 0
#define COMMSERR_WRITE_NOT_CONNECTED 1
#define COMMSERR_PROTOCOL_ERROR 2
#define COMMSERR_CONNECTED 3
#define COMMSERR_DISCONNECTED 4

struct CommunicationInfo {
    uint16_t remoteNo: 4;
    uint16_t connected: 1;
    uint16_t errorMode: 8;
};

/**
 * A callback function that will receive information about comms channels. This is registered as a 
 * static on the TagValueTransport object, and will receive updates for all remote tag value connections.
 */
typedef void (*CommsCallbackFn)(CommunicationInfo);

// forward reference.
class AuthenticationManager;

/**
 * The definition of a transport that can send and receive information remotely using the TagVal protocol.
 * Implementations include SerialTransport and EthernetTransport located in the remotes directory.
 */
class TagValueTransport {
protected:
	FieldAndValue currentField;
public:
	TagValueTransport();
	virtual ~TagValueTransport() {}

	void startMsg(uint16_t msgType);
	void writeField(uint16_t field, const char* value);
	void writeFieldInt(uint16_t field, int value);
    void writeFieldLong(uint16_t field, long value);
	void endMsg();
	FieldAndValue* fieldIfAvailable();
	void clearFieldStatus(FieldValueType ty = FVAL_PROCESSING);

	virtual void flush() = 0;
	virtual int writeChar(char data) = 0;
	virtual int writeStr(const char* data) = 0;
	virtual uint8_t readByte()=0;
	virtual bool readAvailable()=0;

	virtual bool available() = 0;
	virtual bool connected() = 0;
	virtual void close() = 0;

private:
	bool findNextMessageStart();
	bool processMsgKey();
	bool processValuePart();
};

#define FLAG_CURRENTLY_CONNECTED 0
#define FLAG_BOOTSTRAP_MODE 1
#define FLAG_BOOTSTRAP_COMPLETE 2
#define FLAG_AUTHENTICATED 3
#define FLAG_PAIRING_MODE 4

/**
 * The remote connector is what we would normally interact with when dealing with a remote. It provides functionality
 * at the message processing level, for sending messages and processing incoming ones.
 */
class TagValueRemoteConnector {
private:
	const ConnectorLocalInfo* localInfoPgm;
	uint16_t ticksLastSend;
	uint16_t ticksLastRead;
	CombinedMessageProcessor* processor;
	TagValueTransport* transport;	
    CommsCallbackFn commsCallback;
    AuthenticationManager* authManager;

    // used by bootstrapping and writing out messages
    MenuItemIterator iterator;
    MenuItemTypePredicate bootPredicate;
    RemoteNoMenuItemPredicate remotePredicate;

	// the remote connection details take 16 bytes
	char remoteName[16];
	uint8_t remoteMajorVer, remoteMinorVer;
	ApiPlatform remotePlatform;
	uint8_t flags;
	uint8_t remoteNo;
public:
	/**
	 * Construct an instance/
	 * @param transport the actual underlying transport
	 * @param remoteNo the index of this connector, 0 based.
	 */
	TagValueRemoteConnector(uint8_t remoteNo = 0);

    /**
     * Initialises the connector with a specific transport that can send and recevie data, a message processor that can
     * process incoming message, the remote number that should be used and it's name.
     * @param transport a class that implements TagValueTransport for sending and receving data.
     * @param processor a linked list of processors that can process incoming messages.
     * @param localInfoPgm the name and UUID of this local device (in program memory where available).
     */
    void initialise(TagValueTransport* transport, CombinedMessageProcessor* processor, const ConnectorLocalInfo* localInfoPgm);

    void setAuthManager(AuthenticationManager* mgr) { authManager = mgr; }

    /** 
     * If you want to be informed of communication events for this remote, pass a function
     * that takes a CommunicationInfo structure as its parameter.
     */
    void setCommsNotificationCallback(CommsCallbackFn callback) { this->commsCallback = callback; }

	/**
	 * Indicates if the underlying transport is functionality
	 */
	bool isTransportAvailable() { return transport->available(); }

    void provideAuthentication(const char* auth);

	/**
	 *  Indicates if the underlying transport is connected.
	 */
	bool isTransportConnected() { return transport->connected(); }

	/**
	 * Encode a join message onto the wire, giving local name
	 * @param localName the name to send in the join message
	 */
	void encodeJoin();

    /**
     * Encodes a dialog message that the UI can use to render / remove a dialog from the display.
     * @param mode either 'S'how, 'H'ide or 'A'ction
     * @param btn1 the status of the first button
     * @param btn2 the status of the second button
     * @param hdrPgm the header of the dialog (progrma memory)
     * @param buffer the additional text
     */
    void encodeDialogMsg(uint8_t mode, uint8_t btn1, uint8_t btn2, const char* hdrPgm, const char* buffer);

	/**
	 * Encode a bootstrap message indicating we are sending state
	 * @param isComplete true - end of boot sequence.
	 */
	void encodeBootstrap(bool isComplete);

	/**
	 * Encodes a heartbeat message onto the transport
	 */
	void encodeHeartbeat();

	/**
	 * Encodes a bootstrap for an Analog menu item, this gives all needed state
	 * to the remote. 
	 * @param parentId the parent menu
	 * @param item the item to be bootstrapped.
	 */
	void encodeAnalogItem(int parentId, AnalogMenuItem* item);

	/**
	 * Encodes a bootstrap for a Sub menu item, this gives all needed state
	 * to the remote. 
	 * @param parentId the parent menu
	 * @param item the item to be bootstrapped.
	 */
	void encodeSubMenu(int parentId, SubMenuItem* item);

	/**
	 * Encodes a bootstrap for a boolean menu item, this gives all needed state
	 * to the remote. 
	 * @param parentId the parent menu
	 * @param item the item to be bootstrapped.
	 */
	void encodeBooleanMenu(int parentId, BooleanMenuItem* item);

	/**
	 * Encodes a bootstrap for an enum menu item, this gives all needed state
	 * to the remote. 
	 * @param parentId the parent menu
	 * @param item the item to be bootstrapped.
	 */
	void encodeEnumMenu(int parentId, EnumMenuItem* item);

	/**
	 * Encodes a bootstrap for an action menu item, this gives all needed state
	 * to the remote. 
	 * @param parentId the parent menu
	 * @param item the item to be bootstrapped.
	 */
	void encodeActionMenu(int parentId, ActionMenuItem* item);

	/**
	 * Encodes a bootstrap for a float menu item, this gives all needed state
	 * to the remote. 
	 * @param parentId the parent menu
	 * @param item the item to be bootstrapped.
	 */
	void encodeFloatMenu(int parentId, FloatMenuItem* item);

	/**
	 * Encodes a bootstrap for a runtime list menu item, this gives all needed state
	 * to the remote.
	 * @param parentId the parent menu
	 * @param item the item to be bootstrapped.
	 */
	void encodeRuntimeMenuItem(int parentId, RuntimeMenuItem* item);

	/**
	 * Encodes a bootstrap for a multiEdit runtime menu item, this gives all needed state
	 * to the remote.
	 * @param parentId the parent menu
	 * @param item the item to be bootstrapped.
	 */
	void encodeMultiEditMenu(int parentId, RuntimeMenuItem* item);

	/**
	 * Encodes a value change message to be sent to the remote. The embedded device
	 * always sends absolute changes out. 
	 * @param parentId the parent menu
	 * @param theItem the item to be bootstrapped.
	 */
	void encodeChangeValue(MenuItem* theItem);

    /**
     * Encodes an acknowledgement back to the other side to indicate the success or failure
     * of an operation.
     * @param correlation the ID to returned to the other side, or 0.
     * @param status the status to be returned.
     */
    void encodeAcknowledgement(uint32_t correlation, AckResponseStatus status);

	/**
	 * Called frequently to perform all functions, this is arranged interally by
	 * registering a taskManager task.
	 */
	void tick();

    /**
     * Puts the system into pairing mode, If the system is in pairing mode already the
     * display is updated. This will present a dialog on the renderer if there is one
     * and the connection will be allowed only when the user selects to pair on the
     * local device.
     * 
     * @param name the name of the remote device
     * @param uuid the UUID to go with the name
     */
    void pairingRequest(const char* name, const char* uuid);

	/**
	 * Called internally to start a bootstrap on new connections
	 */
	void initiateBootstrap();

	/**
	 * Returns the remoteNo (or remote number) for this remote
	 */
	uint8_t getRemoteNo() {return remoteNo;}

	/**
	 * Returns the remote name for this connection. The name of the other side
	 */
	const char* getRemoteName() {return remoteName;}

	/**
	 * returns the major version of the other party API
	 */
	uint8_t getRemoteMajorVer() {return remoteMajorVer;}

	/**
	 * Returns the minor version of the other party API
	 */
	uint8_t getRemoteMinorVer() {return remoteMinorVer;}
	
	/**
	 * Returns the platform of the other party.
	 */
	ApiPlatform getRemotePlatform() {return remotePlatform;}

	/**
	 * Indicates if we are currently connected. Different to transport level connection as
	 * this includes checking for heartbeats.
	 */
	bool isConnected() { return bitRead(flags, FLAG_CURRENTLY_CONNECTED); }

	/**
	 * Sets the remote name, usually only used by the message processor.
	 */
	void setRemoteName(const char* name);

	/**
	 * Sets the remote connection state, again only used by message processor.
	 */
	void setRemoteConnected(uint8_t major, uint8_t minor, ApiPlatform platform);

    /**
     * Notify any listeners of a communication event on this remote, usually used
     * by message processors to indicate an issue 
     * @param commsEventType an error usually defined in RemoteConnector.h
     */
    void commsNotify(uint16_t commsEventType);

	/** close the connection */
	void close() { if(transport->connected()) transport->close(); }

    /** indicates if the connection is yet authenicated */
    bool isAuthenticated() { return bitRead(flags, FLAG_AUTHENTICATED); }
    AuthenticationManager* getAuthManager() { return authManager; }
private:
	void encodeBaseMenuFields(int parentId, MenuItem* item);
    bool prepareWriteMsg(uint16_t msgType);
	void nextBootstrap();
	void performAnyWrites();
	void dealWithHeartbeating();
    /**
     * Sets the connection state for this remote connection. Does not close the underlying transport.
     * Note that is will also notify the callback of the latest state.
     * 
     * @param conn the new connection state
     */
	void setConnected(bool conn);

	bool isBootstrapMode() { return bitRead(flags, FLAG_BOOTSTRAP_MODE); }
	void setBootstrapMode(bool mode) { bitWrite(flags, FLAG_BOOTSTRAP_MODE, mode); }

	bool isBootstrapComplete() { return bitRead(flags, FLAG_BOOTSTRAP_COMPLETE); }
	void setBootstrapComplete(bool mode) { bitWrite(flags, FLAG_BOOTSTRAP_COMPLETE, mode); }

    void setAuthenticated(bool auth) { bitWrite(flags, FLAG_AUTHENTICATED, auth); }

	bool isPairing() { return bitRead(flags, FLAG_PAIRING_MODE); }
    void setPairing(bool pair) { bitWrite(flags, FLAG_PAIRING_MODE, pair); }
};

#endif /* _TCMENU_REMOTECONNECTOR_H_ */
