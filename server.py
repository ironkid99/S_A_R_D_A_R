import paho.mqtt.client as mqtt
from pymongo import MongoClient
from flask import Flask, jsonify
from flask_cors import CORS

app = Flask(__name__)
CORS(app)  # Enable CORS for all routes

# app = Flask(__name__)

# MongoDB Atlas Cluster URI
MONGODB_URI = "mongodb+srv://sp841117:Shreya%40123@cluster0.10f5onk.mongodb.net/todolistDB"

# Create a MongoClient
client = MongoClient(MONGODB_URI)

# Access the MongoDB Atlas cluster database and collection
db = client['mqtt_data']  # Replace 'mqtt_data' with your actual database name
collection = db['messages']  # Replace 'messages' with your actual collection name

# MQTT Settingskj
MQTT_BROKER = "test.mosquitto.org"
MQTT_PORT = 1883
MQTT_TOPIC = "baby"

# MQTT Callbacks
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe(MQTT_TOPIC)

def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))
    # Save message to MongoDB
    save_to_mongodb(msg.topic, msg.payload)

def save_to_mongodb(topic, payload):
    data = {
        payload.decode('utf-8')  # Assuming the payload is in UTF-8 format
    }
    collection.insert_one(data)

# MQTT Client Setup
client_mqtt = mqtt.Client()
client_mqtt.on_connect = on_connect
client_mqtt.on_message = on_message

# Flask API route to fetch data
@app.route('/data', methods=['GET'])
def get_data():
    # Fetch data from MongoDB
    data_from_mongo = list(collection.find({}, {'_id': 0}))
    return jsonify(data_from_mongo)

if __name__ == '__main__':
    # Connect to MQTT broker
    client_mqtt.connect(MQTT_BROKER, MQTT_PORT, 60)
    # Start MQTT client loop
    client_mqtt.loop_start()
    # Run Flask app
    app.run(debug=True)

# import paho.mqtt.client as mqtt
# from pymongo import MongoClient
# from flask import Flask, jsonify
# from flask_cors import CORS

# app = Flask(__name__)
# CORS(app)  # Enable CORS for all routes

# # MongoDB Atlas Cluster URI
# MONGODB_URI = "mongodb+srv://sp841117:Shreya%40123@cluster0.10f5onk.mongodb.net/todolistDB"

# # Create a MongoClient
# client = MongoClient(MONGODB_URI)

# # Access the MongoDB Atlas cluster database and collection
# db = client['mqtt_data']  # Replace 'mqtt_data' with your actual database name
# collection = db['messages']  # Replace 'messages' with your actual collection name

# # MQTT Settings
# MQTT_BROKER = "test.mosquitto.org"
# MQTT_PORT = 1883
# MQTT_TOPIC = "baby"

# # MQTT Callbacks
# def on_connect(client, userdata, flags, rc):
#     print("Connected with result code "+str(rc))
#     client.subscribe(MQTT_TOPIC)

# def on_message(client, userdata, msg):
#     # print(msg.topic+" "+str(msg.payload))
#     # Save message to MongoDB
#     save_to_mongodb(msg.topic, msg.payload)

# def save_to_mongodb(topic, payload):
#     data = {
#         payload.decode('utf-8')  # Assuming the payload is in UTF-8 format
#     }
#     # collection.insert_one(data)

# # MQTT Client Setup
# client_mqtt = mqtt.Client()
# client_mqtt.on_connect = on_connect
# client_mqtt.on_message = on_message

# # Flask API route to fetch data
# @app.route('/data', methods=['GET'])
# def get_data():
#     # Fetch data from MongoDB
#     data_from_mongo = list(collection.find({}, {'_id': 0}))
#     return jsonify(data_from_mongo)

# if __name__ == '__main__':
#     # Connect to MQTT broker
#     client_mqtt.connect(MQTT_BROKER, MQTT_PORT, 60)
#     # Start MQTT client loop
#     client_mqtt.loop_start()
#     # Run Flask app
#     app.run(debug=True)


