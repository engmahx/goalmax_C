#include <mongoc.h>
#include "goalmax.h"

extern "C" {
    #include "MQTTClient.h"
    #include "MQTTClientPersistence.h"
    }

#define ADDRESS     "iot.eclipse.org:1883"
#define CLIENTID    "ExampleClientPub"
#define TOPIC       "goalmaxTest"
#define PAYLOAD     "Hello World Goalmax!"
#define QOS         1
#define TIMEOUT     10000L

using namespace std;

int main (void)
{
	MQTTClient mqttClient;
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
    MQTTClient_message pubmsg = MQTTClient_message_initializer;
    MQTTClient_deliveryToken token;
	int rc;

	MQTTClient_create(&mqttClient, ADDRESS, CLIENTID,
        MQTTCLIENT_PERSISTENCE_NONE, NULL);
    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;

    if ((rc = MQTTClient_connect(mqttClient, &conn_opts)) != MQTTCLIENT_SUCCESS)
    {
        printf("Failed to connect, return code %d\n", rc);
        exit(-1);
    }
    pubmsg.payload = (void*)PAYLOAD;
    pubmsg.payloadlen = strlen(PAYLOAD);
    pubmsg.qos = QOS;
    pubmsg.retained = 0;
    MQTTClient_publishMessage(mqttClient, TOPIC, &pubmsg, &token);
    printf("Waiting for up to %d seconds for publication of %s\n"
            "on topic %s for client with ClientID: %s\n",
            (int)(TIMEOUT/1000), PAYLOAD, TOPIC, CLIENTID);
    rc = MQTTClient_waitForCompletion(mqttClient, token, TIMEOUT);
    printf("Message with delivery token %d delivered\n", token);
    //MQTTClient_disconnect(mqttClient, 10000);
    //MQTTClient_destroy(&mqttClient);

	const char *uri_str = "mongodb://goalmaxAhmed:123456goalmax@ds047865.mlab.com:47865/goalmax";
	mongoc_client_t *client;
	mongoc_database_t *database;
	mongoc_collection_t *collection;
	bson_t *command, reply, *insert;
	bson_t *document;
	bson_error_t error;
	char *str;
	bool retval;
	GoalmaxOutput * GoalmaxOutputData;

	/*
	* Required to initialize libmongoc's internals
	*/
	mongoc_init ();

	
	/*
	* Create a new client instance
	*/
	client = mongoc_client_new (uri_str);


	/*
	* Register the application name so we can track it in the profile logs
	* on the server. This can also be done from the URI (see other examples).
	*/
	mongoc_client_set_appname (client, "connect-example");

	/*
	* Get a handle on the database "db_name" and collection "coll_name"
	*/
	database = mongoc_client_get_database (client, "goalmax");
	collection = mongoc_client_get_collection (client, "goalmax", "goalmaxTestCollection_mahx");

	/*
	* Do work. This example pings the database, prints the result as JSON and
	* performs an insert
	*/
	command = BCON_NEW ("ping", BCON_INT32 (1));

	retval = mongoc_client_command_simple (
	  client, "admin", command, NULL, &reply, &error);

	if (!retval) {
	  fprintf (stderr, "%s\n", error.message);
	  return EXIT_FAILURE;
	}

	str = bson_as_json (&reply, NULL);
	printf ("%s\n", str);

	insert = BCON_NEW ("hello", BCON_UTF8 ("world"));

	if (!mongoc_collection_insert (
		  collection, MONGOC_INSERT_NONE, insert, NULL, &error)) {
	  fprintf (stderr, "%s\n", error.message);
	}

	bson_destroy (insert);
	bson_destroy (&reply);
	bson_destroy (command);
	bson_free (str);

	goalmaxDataProcessingInit();
	cout << "Initialize completed \n";
	this_thread::sleep_for(chrono::milliseconds(3000));
	 

	while(true)
	{
		cout << "Loop \n";
		GoalmaxOutputData = readGoalmaxData();

		cout.precision(9);
		/* // Print error in position
		printf("Player Lat       : %f \n", GoalmaxOutputData->PredictedLat);
		printf("Player Lon       : %f \n", GoalmaxOutputData->PredictedLon);
		printf("Player Alt       : %f \n", GoalmaxOutputData->PredictedAlt);
		printf("Player MPH       : %f \n", GoalmaxOutputData->ResultantMPH);
		printf("Player Heart rate: %f \n", GoalmaxOutputData->HeartRate);
		printf("Player Heading   : %f \n", GoalmaxOutputData->heading);
		printf("Player Steps     : %f \n", GoalmaxOutputData->steps);
		printf("Player position  : %f \n", float(GoalmaxOutputData->position));
		*/
		cout << "Player Lat       :" << GoalmaxOutputData->PredictedLat << "\n";
		cout << "Player Lon       :" << GoalmaxOutputData->PredictedLon << "\n";
		cout << "Player Alt       :" << GoalmaxOutputData->PredictedAlt << "\n";
		cout << "Player MPH       :" << GoalmaxOutputData->ResultantMPH << "\n";
		cout << "Player Heart rate:" << GoalmaxOutputData->HeartRate << "\n";
		cout << "Player Heading   :" << GoalmaxOutputData->heading << "\n";
		cout << "Player Steps     :" << GoalmaxOutputData->steps << "\n";
		cout << "Player position  :" << GoalmaxOutputData->position << "\n";


		document = bson_new();
		BSON_APPEND_INT32(document, "P", GoalmaxOutputData->position);
		BSON_APPEND_DOUBLE(document, "S", GoalmaxOutputData->steps);
		BSON_APPEND_DOUBLE(document, "H", GoalmaxOutputData->heading);
		BSON_APPEND_DOUBLE(document, "R", GoalmaxOutputData->HeartRate);
		BSON_APPEND_DOUBLE(document, "V", GoalmaxOutputData->ResultantMPH);
		BSON_APPEND_DOUBLE(document, "A", GoalmaxOutputData->PredictedAlt);
		BSON_APPEND_DOUBLE(document, "O", GoalmaxOutputData->PredictedLon);
		BSON_APPEND_DOUBLE(document, "T", GoalmaxOutputData->PredictedLat);

		if (!mongoc_collection_insert (
			  collection, MONGOC_INSERT_NONE, document, NULL, &error)) {
		  fprintf (stderr, "INSERTED %s\n", error.message);
		}
		else
		{
			printf("Mongo ok\n");
		}

		char *documentStr = bson_as_json (document, NULL);

		pubmsg.payload = (void*)documentStr;
		pubmsg.payloadlen = strlen(documentStr);
		pubmsg.qos = QOS;
		pubmsg.retained = 0;
		MQTTClient_publishMessage(mqttClient, TOPIC, &pubmsg, &token);
		printf("Waiting for up to %d seconds for publication of %s\n"
				"on topic %s for client with ClientID: %s\n",
				(int)(TIMEOUT/1000), PAYLOAD, TOPIC, CLIENTID);
		rc = MQTTClient_waitForCompletion(mqttClient, token, TIMEOUT);
		printf("Message with delivery token %d delivered\n", token);

		bson_destroy (document);

		this_thread::sleep_for(chrono::milliseconds(1000));
	}

	/*
	* Release our handles and clean up MQTT and libmongoc
	*/
	MQTTClient_disconnect(mqttClient, 10000);
	MQTTClient_destroy(&mqttClient);

	
	mongoc_collection_destroy (collection);
	mongoc_database_destroy (database);
	mongoc_client_destroy (client);
	mongoc_cleanup ();

	return 0;
}