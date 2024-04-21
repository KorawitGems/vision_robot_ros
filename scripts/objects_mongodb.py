#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker, MarkerArray
import pymongo

class ObjectDatabase:
    def __init__(self):
        db_host = "127.0.0.1"
        db_port = 27017
        db_name = "object_detection"
        self.client = pymongo.MongoClient(db_host, db_port)
        self.db = self.client[db_name]
        self.collection = self.db["objects"]
        rospy.loginfo("Connected mongodb server")
        
        rospy.Subscriber('/object_markers', MarkerArray, self.markers_callback)
        
    def markers_callback(self, marker_array_msg):
        # Extract object name and position from the Marker message
        for marker in marker_array_msg.markers:
            # Save object name and position to the database
            self.save_to_database(marker.ns, marker.pose.position)
        
    def save_to_database(self, object_name, object_position):
        # Check if collection already exists for the object name
        existing_object = self.collection.find_one({"name": object_name})
        if existing_object:
            # If collection exists, update the existing document with the new position data
            self.collection.update_one({"_id": existing_object["_id"]}, {"$set": {
                'position': {
                    "x": object_position.x,
                    "y": object_position.y,
                    "z": object_position.z
                }
            }})
        else:
            # If collection does not exist, create a new collection and insert the position data
            self.collection.insert_one({
                'name': object_name,
                'position': {
                    "x": object_position.x,
                    "y": object_position.y,
                    "z": object_position.z
                }
            })

if __name__ == '__main__':
    try:
        rospy.init_node('objects_database')
        object_db = ObjectDatabase()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
